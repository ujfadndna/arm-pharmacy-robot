"""
AprilTag + QR码 药品识别示例 - MaixCam2端
结合AprilTag定位和QR码信息识别

功能：
1. AprilTag: 提供药盒的6D位姿（用于机械臂抓取）
2. QR码: 提供药品详细信息（名称、有效期等）
"""

from maix import camera, display, image
import struct
import time
import json

# ========== 配置 ==========
CAM_WIDTH = 320
CAM_HEIGHT = 240
APRILTAG_FAMILY = image.ApriltagFamilies.TAG36H11

# 手眼偏移量（需要标定后填入）
CAMERA_TO_GRIPPER = {
    'x': 0,
    'y': 0,
    'z': 50,
}

# 药品数据库（AprilTag ID → 药品信息）
# 实际使用时从QR码读取
DRUG_DATABASE = {
    1: {'name': '阿莫西林', 'shelf': 'A1', 'expiry': '2026-12'},
    2: {'name': '布洛芬', 'shelf': 'A2', 'expiry': '2026-06'},
    3: {'name': '维生素C', 'shelf': 'B1', 'expiry': '2027-03'},
}


# ========== 坐标转换 ==========
def camera_to_gripper(cam_x, cam_y, cam_z):
    """相机坐标系 → 夹爪坐标系"""
    grip_x = cam_z + CAMERA_TO_GRIPPER['x']
    grip_y = -cam_x + CAMERA_TO_GRIPPER['y']
    grip_z = -cam_y + CAMERA_TO_GRIPPER['z']
    return grip_x, grip_y, grip_z


# ========== QR码解析 ==========
def parse_drug_qr(payload):
    """
    解析药品QR码内容

    QR码格式建议：
    {"id":1,"name":"阿莫西林","exp":"2026-12","batch":"20240101"}
    """
    try:
        data = json.loads(payload)
        return {
            'id': data.get('id', 0),
            'name': data.get('name', '未知'),
            'expiry': data.get('exp', '未知'),
            'batch': data.get('batch', ''),
        }
    except:
        # 如果不是JSON，直接返回原始内容
        return {
            'id': 0,
            'name': payload,
            'expiry': '未知',
            'batch': '',
        }


# ========== 主程序 ==========
def main():
    print("=" * 50)
    print("AprilTag + QR码 药品识别")
    print("=" * 50)

    cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT)
    disp = display.Display()

    # QR码检测器（硬件加速）
    try:
        qr_detector = image.QRCodeDetector()
        print("[QR] 使用硬件加速检测器")
    except:
        qr_detector = None
        print("[QR] 使用软件检测器")

    print()
    print("开始检测...")

    while True:
        img = cam.read()

        # ========== AprilTag检测 ==========
        tags = img.find_apriltags(families=APRILTAG_FAMILY)

        for tag in tags:
            tag_id = tag.id()
            cx, cy = tag.cx(), tag.cy()

            # 获取3D位置
            x_mm = tag.x_translation() * 1000
            y_mm = tag.y_translation() * 1000
            z_mm = tag.z_translation() * 1000

            # 转换到夹爪坐标系
            grip_x, grip_y, grip_z = camera_to_gripper(x_mm, y_mm, z_mm)

            # 查询药品信息
            drug_info = DRUG_DATABASE.get(tag_id, {'name': f'药品{tag_id}', 'shelf': '?', 'expiry': '?'})

            # 显示信息
            info1 = f"[{tag_id}] {drug_info['name']}"
            info2 = f"位置: ({grip_x:.0f},{grip_y:.0f},{grip_z:.0f})"
            img.draw_string(cx - 60, cy - 40, info1, image.COLOR_GREEN)
            img.draw_string(cx - 60, cy - 20, info2, image.COLOR_GREEN)

            # 画框
            corners = tag.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                             corners[(i+1)%4][0], corners[(i+1)%4][1],
                             image.COLOR_RED, 2)

            print(f"[AprilTag] ID={tag_id}, {drug_info['name']}, 位置=({grip_x:.1f}, {grip_y:.1f}, {grip_z:.1f})mm")

        # ========== QR码检测 ==========
        if qr_detector:
            qrcodes = qr_detector.detect(img)
        else:
            qrcodes = img.find_qrcodes()

        for qr in qrcodes:
            payload = qr.payload()
            rect = qr.rect()

            # 解析药品信息
            drug_info = parse_drug_qr(payload)

            # 显示信息
            info = f"{drug_info['name']} ({drug_info['expiry']})"
            img.draw_string(rect[0], rect[1] - 20, info, image.COLOR_BLUE)
            img.draw_rectangle(rect[0], rect[1], rect[2], rect[3], image.COLOR_BLUE, 2)

            print(f"[QR码] {drug_info['name']}, 有效期: {drug_info['expiry']}")

        # 显示
        disp.show(img)
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n退出")
