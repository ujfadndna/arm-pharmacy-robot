"""
手眼标定工具 v2 - MaixCam2 端（固定高度版本）

标定原理：
  当吸嘴尖端精确对准 Tag 中心时，Tag 在相机坐标系中的 x/y 偏移
  近似等于相机光轴到吸嘴中心的固定偏移量（camera -> tool 向量）。
  多次采样取均值即为标定结果。

操作要点：
  1. Tag 固定在桌面，不要手动移动 Tag。
  2. 机械臂在标定开始前设定工作高度，整个过程保持该高度（Z 不变）。
  3. 每次采样都让吸嘴对准 Tag 中心并静止 3 秒，系统自动记录。
  4. 组间仅做 XY 平移（前后/左右），不要升降 Z。
  5. 采样完成后将结果填入 visual_servo.py 的 OFFSET_X / OFFSET_Y。
"""

from maix import camera, display, image
import time
import json
import traceback
import math

# ========== 配置 ==========
CAM_WIDTH  = 320
CAM_HEIGHT = 240
APRILTAG_FAMILY = image.ApriltagFamilies.TAG36H11

TAG_SIZE_M = 0.04        # Tag物理边长（米），必须与实际打印尺寸一致

SAMPLES_NEEDED   = 10    # 采集满这么多组后自动结束
STABLE_SECONDS   = 3.0   # 静止保持几秒触发记录
STABLE_Z_TOL_MM  = 4.0   # Z变化在此范围内视为"静止"
MIN_Z_MM         = 10.0  # Z太小说明tag紧贴镜头，跳过
COOLDOWN_SECONDS = 8.0   # 记录后给用户重新对准的时间（秒）
XY_SHIFT_MIN_MM  = 10.0  # 每组之间建议XY平移最小距离
XY_SHIFT_MAX_MM  = 30.0  # 每组之间建议XY平移最大距离
ENABLE_Z_LOCK    = False # 是否启用跨样本Z锁定（斜视相机建议关闭）
Z_LOCK_TOL_MM    = 35.0  # 相对首样本的Z锁定范围（mm）
XY_JUMP_RADIUS_MM   = 35.0  # 相邻样本二维跳变半径阈值（mm）
XY_MEDIAN_RADIUS_MM = 30.0  # 相对历史中位数二维偏移阈值（mm）
XY_FILTER_START_N   = 4     # 至少采样这么多组后再做XY离群拒绝
HINT_INTERVAL_S  = 4.0   # 阶段提示最小间隔（秒）

# 固定工作高度（吸嘴距Tag表面的距离，单位mm）
# 设置为你实际取药时的高度，标定只在此高度进行
WORK_HEIGHT_MM   = 100.0

CX = CAM_WIDTH  // 2
CY = CAM_HEIGHT // 2


def median(values):
    vals = sorted(values)
    n = len(vals)
    m = n // 2
    if n % 2 == 1:
        return vals[m]
    return 0.5 * (vals[m - 1] + vals[m])


def main():
    print("=" * 55)
    print(f"  手眼标定 v2  (TAG36H11, {TAG_SIZE_M*1000:.0f}mm)")
    print("=" * 55)
    print()
    print("操作步骤（按阶段执行）：")
    print("  阶段A-准备")
    print("    1) 将Tag平放桌面并固定，不要手动移动Tag")
    print(f"    2) 调整机械臂高度，让吸嘴距Tag约 {WORK_HEIGHT_MM:.0f}mm")
    print("    3) 后续全程保持该高度不变（不要升降Z）")
    print("  阶段B-采样")
    print("    4) 手动让吸嘴精确对准Tag中心，保持不动 -> 3秒自动记录")
    print(f"    5) 每次记录后，只在XY平面平移约{XY_SHIFT_MIN_MM:.0f}~{XY_SHIFT_MAX_MM:.0f}mm")
    print("       (前后/左右移动)，不要升降Z高度")
    print(f"    6) 重新对准Tag中心并静止3秒，重复直到采满{SAMPLES_NEEDED}组")
    print()
    print(f"[CFG] TAG={TAG_SIZE_M*1000:.0f}mm  "
          f"分辨率={CAM_WIDTH}x{CAM_HEIGHT}  目标样本数={SAMPLES_NEEDED}")
    print(f"[CFG] 固定高度={WORK_HEIGHT_MM:.0f}mm  "
          f"组间平移=仅XY {XY_SHIFT_MIN_MM:.0f}~{XY_SHIFT_MAX_MM:.0f}mm")
    if ENABLE_Z_LOCK:
        print(f"[CFG] Z锁定=首样本±{Z_LOCK_TOL_MM:.0f}mm  "
              f"XY跳变半径={XY_JUMP_RADIUS_MM:.0f}mm")
    else:
        print(f"[CFG] Z锁定=关闭(斜视相机推荐)  "
              f"XY跳变半径={XY_JUMP_RADIUS_MM:.0f}mm")
    print()

    cam  = camera.Camera(CAM_WIDTH, CAM_HEIGHT)
    disp = display.Display()

    samples              = []
    stable_start         = None
    last_z_mm            = None
    recorded_this_stable = False
    cooldown_start       = None
    z_ref_mm             = None
    reject_msg           = ""
    reject_msg_until     = 0.0
    hint_last_time       = {}

    def stage_hint(key, msg, now):
        last = hint_last_time.get(key, -1e9)
        if (now - last) >= HINT_INTERVAL_S:
            print(msg)
            hint_last_time[key] = now

    print("开始标定，等待检测Tag...")
    print("[提示] 平移=机械臂在XY平面移动，不是调整高度Z。")
    print(f"[提示] 当前要求：Tag固定不动，机械臂保持高度约{WORK_HEIGHT_MM:.0f}mm。")
    print()

    while True:
        img = cam.read()
        t   = time.time()

        tags = img.find_apriltags(families=APRILTAG_FAMILY)

        # 画图像中心十字
        img.draw_line(CX - 25, CY, CX + 25, CY, image.COLOR_YELLOW, 1)
        img.draw_line(CX, CY - 25, CX, CY + 25, image.COLOR_YELLOW, 1)

        if not tags:
            stable_start         = None
            last_z_mm            = None
            recorded_this_stable = False

            stage_hint("no_tag", "[阶段提示] 未检测到Tag：仅移动机械臂把Tag放入画面，Tag本体保持固定不动。", t)

            img.draw_string(10, 10, "未检测到 Tag",            image.COLOR_RED)
            img.draw_string(10, 30, "仅移动机械臂找Tag",       image.COLOR_WHITE)
            img.draw_string(10, 50, f"Samples: {len(samples)}/{SAMPLES_NEEDED}", image.COLOR_WHITE)
            disp.show(img)
            time.sleep(0.05)
            continue

        tag = tags[0]
        try:
            tag_id = tag.id()
            x_mm   = tag.x_translation() * TAG_SIZE_M * 1000
            y_mm   = tag.y_translation() * TAG_SIZE_M * 1000
            z_mm   = tag.z_translation() * TAG_SIZE_M * 1000
            tcx    = tag.cx()
            tcy    = tag.cy()
        except Exception as e:
            print(f"[ERR] tag读取异常: {e}")
            disp.show(img)
            time.sleep(0.05)
            continue

        # 画tag边框和中心
        try:
            corners = tag.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                              corners[(i+1)%4][0], corners[(i+1)%4][1],
                              image.COLOR_RED, 2)
        except Exception:
            pass
        img.draw_circle(tcx, tcy, 6, image.COLOR_BLUE, -1)

        # ---- 稳定性检测 ----
        if last_z_mm is not None and abs(z_mm - last_z_mm) > STABLE_Z_TOL_MM:
            stable_start         = None
            recorded_this_stable = False
        last_z_mm = z_mm

        if stable_start is None:
            stable_start = t

        stable_elapsed = t - stable_start
        remaining      = max(0.0, STABLE_SECONDS - stable_elapsed)

        # ---- 固定高度(Z锁定)检查 ----
        z_lock_ok  = True
        z_lock_err = 0.0
        if ENABLE_Z_LOCK and z_ref_mm is not None:
            z_lock_err = abs(z_mm - z_ref_mm)
            if z_lock_err > Z_LOCK_TOL_MM:
                z_lock_ok = False
                stable_start         = None
                recorded_this_stable = False

        # ---- 冷却期检查 ----
        in_cooldown        = False
        cooldown_remaining = 0.0
        if cooldown_start is not None:
            cooldown_remaining = max(0.0, COOLDOWN_SECONDS - (t - cooldown_start))
            if cooldown_remaining > 0:
                in_cooldown = True
            else:
                # 冷却结束后，重置稳定窗口，允许下一组样本重新计时
                cooldown_start       = None
                stable_start         = None
                recorded_this_stable = False

        # ---- 触发记录 ----
        if (not recorded_this_stable
                and not in_cooldown
                and stable_elapsed >= STABLE_SECONDS
                and abs(z_mm) >= MIN_Z_MM
                and (not ENABLE_Z_LOCK or z_lock_ok)):
            reject_reason = None
            if len(samples) >= XY_FILTER_START_N:
                dx_last = abs(x_mm - samples[-1]['x_mm'])
                dy_last = abs(y_mm - samples[-1]['y_mm'])
                jump_r = math.sqrt(dx_last * dx_last + dy_last * dy_last)
                med_x = median([s['x_mm'] for s in samples])
                med_y = median([s['y_mm'] for s in samples])
                dx_med = abs(x_mm - med_x)
                dy_med = abs(y_mm - med_y)
                med_r = math.sqrt(dx_med * dx_med + dy_med * dy_med)
                # 只有同时满足“相邻跳变大 + 偏离历史大”才拒绝，避免轻微平移导致卡死
                if jump_r > XY_JUMP_RADIUS_MM and med_r > XY_MEDIAN_RADIUS_MM:
                    reject_reason = (f"二维偏移过大 (jump={jump_r:.1f}, median={med_r:.1f}, "
                                     f"阈值={XY_JUMP_RADIUS_MM:.1f}/{XY_MEDIAN_RADIUS_MM:.1f})")

            if reject_reason is not None:
                reject_msg = "REJECT: X/Y突变过大, 本组不计数"
                reject_msg_until = t + 2.0
                stable_start = None
                stage_hint("xy_reject",
                           f"[REJECT] {reject_reason}，本组不计数。请重新对准后重采。",
                           t)
            else:
                if ENABLE_Z_LOCK and z_ref_mm is None:
                    z_ref_mm = z_mm
                    print(f"[INFO] Z锁定基准已建立: {z_ref_mm:+.1f} mm (允许±{Z_LOCK_TOL_MM:.0f}mm)")

                n = len(samples) + 1
                samples.append({
                    'n':    n,
                    'x_mm': x_mm,
                    'y_mm': y_mm,
                    'z_mm': z_mm,
                })
                recorded_this_stable = True
                cooldown_start       = t
                print(f"[样本 {n:2d}/{SAMPLES_NEEDED}] "
                      f"X={x_mm:+6.1f}  Y={y_mm:+6.1f}  Z={z_mm:+6.1f} mm")
                print(f"         下一步：仅XY平移{XY_SHIFT_MIN_MM:.0f}~{XY_SHIFT_MAX_MM:.0f}mm，"
                      "保持Z高度不变；再对准中心并静止3秒。")

        # ---- 屏幕 UI ----
        img.draw_string(10, 10, f"ID:{tag_id}  Z:{z_mm:.0f}mm",              image.COLOR_GREEN)
        img.draw_string(10, 30, f"X:{x_mm:+.1f}  Y:{y_mm:+.1f} mm",         image.COLOR_GREEN)
        img.draw_string(10, 50, f"Samples: {len(samples)}/{SAMPLES_NEEDED}",  image.COLOR_WHITE)

        if in_cooldown:
            stage_hint(
                f"cooldown_{len(samples)}",
                f"[阶段提示] 已记录{len(samples)}组：仅XY平移{XY_SHIFT_MIN_MM:.0f}~{XY_SHIFT_MAX_MM:.0f}mm，"
                "不要改Z高度，然后重新对准Tag中心。",
                t
            )
            img.draw_string(10, 75, f"已记录! 仅XY平移, Z不变  {cooldown_remaining:.0f}s", image.COLOR_GREEN)
        elif t < reject_msg_until:
            img.draw_string(10, 75, reject_msg, image.COLOR_RED)
        elif recorded_this_stable:
            img.draw_string(10, 75, "已记录! 请只移动XY到下一点",          image.COLOR_GREEN)
        elif abs(z_mm) < MIN_Z_MM:
            stage_hint("z_too_small", "[阶段提示] 当前Z过小，请抬高机械臂到设定高度后再继续。", t)
            img.draw_string(10, 75, f"Z太小({z_mm:.0f}mm)，请抬高机械臂",    image.COLOR_RED)
        elif ENABLE_Z_LOCK and not z_lock_ok:
            stage_hint("z_lock_out",
                       f"[阶段提示] Z偏离基准 {z_lock_err:.1f}mm（允许±{Z_LOCK_TOL_MM:.0f}mm），请回到原高度。",
                       t)
            img.draw_string(10, 75, f"Z偏离基准{z_lock_err:.0f}mm, 请回到原高度", image.COLOR_RED)
        else:
            stage_hint("align_hold", "[阶段提示] 对准Tag中心并保持静止3秒，平移指XY移动，不是升降Z。", t)
            img.draw_string(10, 75, f"保持静止...  {remaining:.1f}s",         image.COLOR_WHITE)

        img.draw_string(10, CAM_HEIGHT - 25,
                        "平移=XY移动(10~30mm), 不是升降Z",
                        image.COLOR_WHITE)

        disp.show(img)

        if len(samples) >= SAMPLES_NEEDED:
            print()
            print("[INFO] 采集完成，正在计算结果...")
            time.sleep(0.5)
            break

        time.sleep(0.05)

    return samples


def calculate(samples):
    if not samples:
        print("无标定数据，退出。")
        return

    n  = len(samples)
    ax = sum(s['x_mm'] for s in samples) / n
    ay = sum(s['y_mm'] for s in samples) / n
    az = sum(s['z_mm'] for s in samples) / n
    sx = (sum((s['x_mm'] - ax) ** 2 for s in samples) / n) ** 0.5
    sy = (sum((s['y_mm'] - ay) ** 2 for s in samples) / n) ** 0.5

    print()
    print("=" * 55)
    print(f"  标定结果（{n} 组样本）")
    print("=" * 55)
    print(f"  相机->吸嘴偏移（相机坐标系）：")
    print(f"    X = {ax:+.1f} mm   (标准差 {sx:.1f} mm)")
    print(f"    Y = {ay:+.1f} mm   (标准差 {sy:.1f} mm)")
    print(f"    Z = {az:+.1f} mm   (平均工作距离)")
    print()
    if sx < 2.0 and sy < 2.0:
        print("  [OK] 标准差 < 2mm，标定质量良好")
    else:
        print("  [WARN] 标准差偏大，建议重新标定")
        print("         原因：每次吸嘴未精确对准Tag中心")
    print()
    print("  将以下两行填入 visual_servo.py：")
    print()
    print(f"    OFFSET_X = {ax:.1f}   # 相机->吸嘴 X 偏移 (mm)")
    print(f"    OFFSET_Y = {ay:.1f}   # 相机->吸嘴 Y 偏移 (mm)")
    print()
    print("=" * 55)

    result = {
        'samples':           n,
        'tool_offset_x_mm':  ax,
        'tool_offset_y_mm':  ay,
        'tool_offset_z_mm':  az,
        'std_x_mm':          sx,
        'std_y_mm':          sy,
        'raw_data':          samples,
    }
    try:
        with open('/root/hand_eye_calibration.json', 'w') as f:
            json.dump(result, f, indent=2)
        print("  已保存: /root/hand_eye_calibration.json")
    except Exception as e:
        print(f"  [WARN] 保存失败: {e}")


if __name__ == "__main__":
    samples = []
    try:
        samples = main()
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
    except Exception as e:
        print(f"\n[FATAL] 未预期异常: {e}")
        traceback.print_exc()
    finally:
        calculate(samples)
