"""
apriltag_detect.py - MaixCam2 AprilTag识别 (官方API风格)

功能：
1. 检测TAG36H11家族的AprilTag
2. 显示ID、位置、旋转角度
3. 通过UART发送给RA6M5

UART接线 (MaixCam2):
  A21 (UART4_TX) -> RA6M5 RX
  A22 (UART4_RX) -> RA6M5 TX
"""

from maix import camera, display, image, app, uart, pinmap, time, sys, err

# ========== 配置 ==========
# 注意：分辨率太高会导致内存不足，AprilTag检测建议用小分辨率
CAM_WIDTH = 320
CAM_HEIGHT = 240

# AprilTag配置
FAMILIES = image.ApriltagFamilies.TAG36H11
TAG_SIZE_MM = 30.0  # AprilTag实际尺寸 (mm)

# 相机内参 (需要标定，-1表示自动)
FX = -1.0
FY = -1.0
CX = CAM_WIDTH // 2
CY = CAM_HEIGHT // 2

# ROI区域 (可选，None表示全图)
ROI = None  # 或 [x, y, w, h]

# 手眼偏移 (相机到基座)
OFFSET_X = 0.0
OFFSET_Y = 0.0
OFFSET_Z = 50.0

# 稳定性过滤配置
STABLE_THRESHOLD = 3  # 连续检测到N帧才算有效
tag_detect_count = {}  # {tag_id: count}
tag_lost_count = {}    # {tag_id: count}


# ========== UART初始化 ==========
def init_uart():
    """初始化UART (MaixCam2用UART4)"""
    device_id = sys.device_id()

    if device_id == "maixcam2":
        pin_function = {
            "A21": "UART4_TX",
            "A22": "UART4_RX"
        }
        device = "/dev/ttyS4"
    else:
        # MaixCam (K210)
        pin_function = {
            "A16": "UART0_TX",
            "A17": "UART0_RX"
        }
        device = "/dev/ttyS0"

    # 设置引脚功能
    for pin, func in pin_function.items():
        err.check_raise(pinmap.set_pin_function(pin, func),
                        "Failed set pin {} to {}".format(pin, func))

    # 初始化UART
    serial = uart.UART(device, 115200)
    print("[UART] {} @ 115200".format(device))
    return serial


# ========== 坐标转换 ==========
def camera_to_base(tx, ty, tz):
    """
    相机坐标 → 基座坐标
    tx, ty, tz: AprilTag的translation (单位: tag尺寸)
    返回: 基座坐标 (mm)
    """
    # 转换为mm
    cam_x = tx * TAG_SIZE_MM
    cam_y = ty * TAG_SIZE_MM
    cam_z = tz * TAG_SIZE_MM

    # 坐标系变换 (根据相机安装方式调整)
    base_x = cam_z + OFFSET_X
    base_y = -cam_x + OFFSET_Y
    base_z = -cam_y + OFFSET_Z

    return base_x, base_y, base_z


# ========== 主程序 ==========
def main():
    print("=" * 40)
    print("  MaixCam2 AprilTag 识别")
    print("  Device: {}".format(sys.device_id()))
    print("=" * 40)

    # 初始化
    cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT)
    disp = display.Display()
    serial = init_uart()

    print("\n开始检测... (Ctrl+C 退出)\n")

    # 稳定性过滤状态
    global tag_detect_count, tag_lost_count

    while not app.need_exit():
        img = cam.read()

        # 检测AprilTag
        if ROI:
            apriltags = img.find_apriltags(ROI, FAMILIES, FX, FY, CX, CY)
        else:
            apriltags = img.find_apriltags(families=FAMILIES)

        # 当前帧检测到的ID
        detected_ids = set()

        for a in apriltags:
            tag_id = a.id()
            detected_ids.add(tag_id)

            # 更新检测计数
            tag_detect_count[tag_id] = tag_detect_count.get(tag_id, 0) + 1
            tag_lost_count[tag_id] = 0  # 重置丢失计数

            # 画角点连线
            corners = a.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                              corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                              image.COLOR_GREEN, 2)

            # 画矩形框
            rect = a.rect()
            img.draw_rect(rect[0], rect[1], rect[2], rect[3],
                          image.COLOR_BLUE, 1)

            # 获取位置
            tx = a.x_translation()
            ty = a.y_translation()
            tz = a.z_translation()
            base_x, base_y, base_z = camera_to_base(tx, ty, tz)

            # 检查是否稳定
            is_stable = tag_detect_count[tag_id] >= STABLE_THRESHOLD

            # 显示ID (稳定显示绿色，不稳定显示红色带问号)
            if is_stable:
                img.draw_string(a.cx() - 20, a.cy() - 40,
                                "ID:{}".format(tag_id), image.COLOR_GREEN)
            else:
                img.draw_string(a.cx() - 20, a.cy() - 40,
                                "ID:{}?".format(tag_id), image.COLOR_RED)

            # 显示位置
            img.draw_string(a.cx() - 50, a.cy() + 20,
                            "({:.0f},{:.0f},{:.0f})".format(base_x, base_y, base_z),
                            image.COLOR_BLUE)

            # 只有稳定后才发送UART和打印
            if is_stable:
                print("[TAG] ID={}, pos=({:.1f},{:.1f},{:.1f})".format(
                    tag_id, base_x, base_y, base_z))
                msg = "TAG:{},{:.1f},{:.1f},{:.1f}\n".format(
                    tag_id, base_x, base_y, base_z)
                serial.write_str(msg)

        # 更新丢失计数 (没检测到的tag)
        for tag_id in list(tag_detect_count.keys()):
            if tag_id not in detected_ids:
                tag_lost_count[tag_id] = tag_lost_count.get(tag_id, 0) + 1
                # 丢失超过3帧，重置检测计数
                if tag_lost_count[tag_id] >= 3:
                    tag_detect_count[tag_id] = 0

        # 显示检测数量 (稳定/总数)
        stable_count = sum(1 for tid in detected_ids
                          if tag_detect_count.get(tid, 0) >= STABLE_THRESHOLD)
        img.draw_string(10, 10, "Tags:{}/{}".format(stable_count, len(apriltags)),
                        image.COLOR_WHITE)

        disp.show(img)
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
