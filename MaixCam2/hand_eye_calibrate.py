"""
手眼偏移量标定辅助工具 - MaixCam2端
用于测量相机到夹爪的固定偏移量

使用方法：
1. 把AprilTag放在已知位置（比如桌面上）
2. 手动控制机械臂，让夹爪对准AprilTag中心
3. 记录此时AprilTag在相机中的位置
4. 这个位置就是相机到夹爪的偏移量

重复几次取平均值，提高精度
"""

from maix import camera, display, image
import time
import json

# ========== 配置 ==========
CAM_WIDTH = 320
CAM_HEIGHT = 240
APRILTAG_FAMILY = image.ApriltagFamilies.TAG36H11

# 标定数据存储
calibration_data = []


def main():
    print("=" * 50)
    print("手眼偏移量标定辅助工具")
    print("=" * 50)
    print()
    print("使用方法：")
    print("1. 把AprilTag放在桌面上")
    print("2. 手动控制机械臂，让夹爪对准AprilTag中心")
    print("3. 按下按钮记录数据（或等待自动记录）")
    print("4. 重复3-5次，取平均值")
    print()

    cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT)
    disp = display.Display()

    sample_count = 0
    last_record_time = 0
    record_interval = 3  # 每3秒自动记录一次（如果检测到标签）

    print("开始标定...")
    print("当夹爪对准AprilTag时，数据会自动记录")
    print("按Ctrl+C结束并计算结果")
    print()

    while True:
        img = cam.read()
        tags = img.find_apriltags(families=APRILTAG_FAMILY)

        current_time = time.time()

        for tag in tags:
            tag_id = tag.id()
            cx, cy = tag.cx(), tag.cy()

            # 获取3D位置
            x = tag.x_translation()
            y = tag.y_translation()
            z = tag.z_translation()

            # 转换为毫米
            x_mm = x * 1000
            y_mm = y * 1000
            z_mm = z * 1000

            # 显示实时数据
            info1 = f"ID:{tag_id}"
            info2 = f"X:{x_mm:.1f}mm"
            info3 = f"Y:{y_mm:.1f}mm"
            info4 = f"Z:{z_mm:.1f}mm"

            img.draw_string(10, 10, info1, image.COLOR_GREEN)
            img.draw_string(10, 30, info2, image.COLOR_GREEN)
            img.draw_string(10, 50, info3, image.COLOR_GREEN)
            img.draw_string(10, 70, info4, image.COLOR_GREEN)

            # 画框和中心
            corners = tag.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                             corners[(i+1)%4][0], corners[(i+1)%4][1],
                             image.COLOR_RED, 2)
            img.draw_circle(cx, cy, 5, image.COLOR_BLUE, -1)

            # 自动记录（每隔一段时间）
            if current_time - last_record_time > record_interval:
                if abs(z_mm) > 10:  # 确保检测有效
                    sample_count += 1
                    calibration_data.append({
                        'sample': sample_count,
                        'tag_id': tag_id,
                        'x_mm': x_mm,
                        'y_mm': y_mm,
                        'z_mm': z_mm,
                    })
                    last_record_time = current_time

                    print(f"[样本 {sample_count}] Tag={tag_id}, X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f} mm")

        # 显示样本数
        img.draw_string(10, CAM_HEIGHT - 30, f"Samples: {sample_count}", image.COLOR_WHITE)

        disp.show(img)
        time.sleep(0.05)


def calculate_result():
    """计算标定结果"""
    if len(calibration_data) == 0:
        print("没有标定数据！")
        return

    print()
    print("=" * 50)
    print("标定结果")
    print("=" * 50)

    # 计算平均值
    avg_x = sum(d['x_mm'] for d in calibration_data) / len(calibration_data)
    avg_y = sum(d['y_mm'] for d in calibration_data) / len(calibration_data)
    avg_z = sum(d['z_mm'] for d in calibration_data) / len(calibration_data)

    # 计算标准差
    std_x = (sum((d['x_mm'] - avg_x)**2 for d in calibration_data) / len(calibration_data)) ** 0.5
    std_y = (sum((d['y_mm'] - avg_y)**2 for d in calibration_data) / len(calibration_data)) ** 0.5
    std_z = (sum((d['z_mm'] - avg_z)**2 for d in calibration_data) / len(calibration_data)) ** 0.5

    print(f"样本数: {len(calibration_data)}")
    print()
    print("平均偏移量（相机坐标系）：")
    print(f"  X: {avg_x:.2f} mm (std: {std_x:.2f})")
    print(f"  Y: {avg_y:.2f} mm (std: {std_y:.2f})")
    print(f"  Z: {avg_z:.2f} mm (std: {std_z:.2f})")
    print()
    print("将以下值填入 hand_eye_simple.py 的 CAMERA_TO_GRIPPER：")
    print()
    print("CAMERA_TO_GRIPPER = {")
    print(f"    'x': {avg_x:.1f},  # 相机X方向偏移")
    print(f"    'y': {avg_y:.1f},  # 相机Y方向偏移")
    print(f"    'z': {avg_z:.1f},  # 相机Z方向偏移")
    print("}")
    print()

    # 保存到文件
    result = {
        'samples': len(calibration_data),
        'offset_x_mm': avg_x,
        'offset_y_mm': avg_y,
        'offset_z_mm': avg_z,
        'std_x': std_x,
        'std_y': std_y,
        'std_z': std_z,
        'raw_data': calibration_data,
    }

    with open('/root/hand_eye_calibration.json', 'w') as f:
        json.dump(result, f, indent=2)
    print("标定数据已保存到 /root/hand_eye_calibration.json")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        calculate_result()
