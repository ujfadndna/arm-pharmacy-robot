"""
简化版手眼坐标转换 - MaixCam2端
Eye-in-Hand配置：相机装在机械臂末端

原理：
1. AprilTag检测 → 得到目标在相机坐标系下的位置
2. 固定偏移量转换 → 得到目标在夹爪坐标系下的位置
3. 发送给RA6M5 → 机械臂执行抓取

注意：这是简化版，用固定偏移量代替精确的手眼标定矩阵
"""

from maix import camera, display, image
import struct
import serial
import math
import time

# ========== 配置参数 ==========

# 相机分辨率
CAM_WIDTH = 320
CAM_HEIGHT = 240

# AprilTag系列
APRILTAG_FAMILY = image.ApriltagFamilies.TAG36H11

# AprilTag实际尺寸（米）- 用于距离估算
TAG_SIZE_M = 0.05  # 5cm

# 手眼偏移量（毫米）- 需要实际测量！
# 这是相机坐标系到夹爪坐标系的固定偏移
# X: 相机光轴方向（正=前）
# Y: 相机水平方向（正=右）
# Z: 相机垂直方向（正=下）
CAMERA_TO_GRIPPER = {
    'x': 0,      # 相机和夹爪在同一垂直线上
    'y': 0,      # 没有左右偏移
    'z': 50,     # 相机比夹爪高50mm（夹爪在相机下方）
}

# 串口配置（连接RA6M5）
UART_PORT = "/dev/ttyS1"  # MaixCam2的串口
UART_BAUD = 115200

# 通信协议
FRAME_HEADER = 0xAA55
CMD_TARGET_POS = 0x10  # 发送目标位置命令


# ========== 坐标转换 ==========

def camera_to_gripper(cam_x, cam_y, cam_z):
    """
    将相机坐标系下的位置转换到夹爪坐标系

    相机坐标系（AprilTag返回的）：
    - X: 水平向右
    - Y: 垂直向下
    - Z: 光轴方向（向前）

    夹爪坐标系（机械臂使用的）：
    - X: 向前
    - Y: 向左
    - Z: 向上

    Args:
        cam_x, cam_y, cam_z: 相机坐标系下的位置（毫米）

    Returns:
        gripper_x, gripper_y, gripper_z: 夹爪坐标系下的位置（毫米）
    """
    # 坐标系转换（根据实际安装方式调整）
    # 这里假设相机正对前方，没有旋转
    gripper_x = cam_z + CAMERA_TO_GRIPPER['x']   # 相机Z → 夹爪X（前）
    gripper_y = -cam_x + CAMERA_TO_GRIPPER['y']  # 相机X → 夹爪Y（左，取反）
    gripper_z = -cam_y + CAMERA_TO_GRIPPER['z']  # 相机Y → 夹爪Z（上，取反）

    return gripper_x, gripper_y, gripper_z


def estimate_distance(tag):
    """
    从AprilTag估算距离

    方法1：使用translation（如果相机已标定）
    方法2：使用标签宽度（简单但不太准）
    """
    # 方法1：使用3D位姿（推荐）
    x = tag.x_translation()
    y = tag.y_translation()
    z = tag.z_translation()

    # 如果translation有效（非零）
    if abs(z) > 0.001:
        # 距离 = sqrt(x² + y² + z²)
        distance = math.sqrt(x*x + y*y + z*z)
        return x * 1000, y * 1000, z * 1000  # 转换为毫米

    # 方法2：使用标签宽度估算（备用）
    # distance = k / width，k需要标定
    k = 5000  # 标定常数，需要实际测量
    width = tag.w()
    if width > 0:
        distance_mm = k / width * 1000
        # 假设标签在相机正前方
        return 0, 0, distance_mm

    return None, None, None


# ========== 通信协议 ==========

def send_target_position(uart, tag_id, x, y, z):
    """
    发送目标位置给RA6M5

    帧格式：
    [Header 2B] [Cmd 1B] [TagID 1B] [X 4B] [Y 4B] [Z 4B] [Checksum 1B]
    """
    # 打包数据
    data = struct.pack('<HBBfff',
                       FRAME_HEADER,      # 帧头
                       CMD_TARGET_POS,    # 命令
                       tag_id,            # 标签ID
                       x, y, z)           # 位置（float）

    # 计算校验和
    checksum = sum(data) & 0xFF
    data += struct.pack('B', checksum)

    # 发送
    uart.write(data)
    print(f"[TX] Tag={tag_id}, Pos=({x:.1f}, {y:.1f}, {z:.1f}) mm")


# ========== 主程序 ==========

def main():
    print("=" * 50)
    print("简化版手眼坐标转换")
    print("Eye-in-Hand配置")
    print("=" * 50)
    print(f"相机偏移: X={CAMERA_TO_GRIPPER['x']}mm, Y={CAMERA_TO_GRIPPER['y']}mm, Z={CAMERA_TO_GRIPPER['z']}mm")
    print()

    # 初始化相机
    cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT)
    disp = display.Display()

    # 初始化串口（可选，如果不连接RA6M5可以注释掉）
    uart = None
    try:
        uart = serial.Serial(UART_PORT, UART_BAUD, timeout=0.1)
        print(f"[UART] Connected to {UART_PORT}")
    except Exception as e:
        print(f"[UART] Not connected: {e}")
        print("[UART] Running in demo mode (no RA6M5)")

    print()
    print("开始检测AprilTag...")
    print("按Ctrl+C退出")
    print()

    while True:
        # 读取图像
        img = cam.read()

        # 检测AprilTag
        tags = img.find_apriltags(families=APRILTAG_FAMILY)

        for tag in tags:
            # 获取标签信息
            tag_id = tag.id()
            cx, cy = tag.cx(), tag.cy()

            # 估算相机坐标系下的位置
            cam_x, cam_y, cam_z = estimate_distance(tag)

            if cam_z is not None:
                # 转换到夹爪坐标系
                grip_x, grip_y, grip_z = camera_to_gripper(cam_x, cam_y, cam_z)

                # 打印信息
                print(f"[Tag {tag_id}]")
                print(f"  相机坐标: ({cam_x:.1f}, {cam_y:.1f}, {cam_z:.1f}) mm")
                print(f"  夹爪坐标: ({grip_x:.1f}, {grip_y:.1f}, {grip_z:.1f}) mm")

                # 发送给RA6M5
                if uart:
                    send_target_position(uart, tag_id, grip_x, grip_y, grip_z)

                # 在图像上显示
                info = f"ID:{tag_id} ({grip_x:.0f},{grip_y:.0f},{grip_z:.0f})"
                img.draw_string(cx - 50, cy - 30, info, image.COLOR_GREEN)

            # 画框
            corners = tag.corners()
            for i in range(4):
                img.draw_line(corners[i][0], corners[i][1],
                             corners[(i+1)%4][0], corners[(i+1)%4][1],
                             image.COLOR_RED, 2)

            # 画中心点
            img.draw_circle(cx, cy, 5, image.COLOR_BLUE, -1)

        # 显示
        disp.show(img)

        # 控制帧率
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n退出")
