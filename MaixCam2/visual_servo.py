"""
visual_servo.py - MaixCam2 视觉伺服模块

功能：
1. 全局扫描模式 - 扫描整个药柜
2. 精定位模式 - 返回目标AprilTag相对于画面中心的偏差

通信协议：
  RA6M5 -> MaixCam2:
    SCAN\n              - 请求全局扫描
    ALIGN:id\n          - 请求精定位，id为目标AprilTag ID

  MaixCam2 -> RA6M5:
    SCAN_START\n        - 扫描开始
    TAG:id,x,y,z\n      - 检测到的Tag
    SCAN_END\n          - 扫描结束
    ALIGN_OK:dx,dy\n    - 精定位成功，返回偏差(mm)
    ALIGN_FAIL\n        - 精定位失败（未检测到目标）
"""

from maix import camera, display, image, app, uart, pinmap, time, sys, err

# ========== 配置 ==========
CAM_WIDTH = 320
CAM_HEIGHT = 240

# AprilTag配置
FAMILIES = image.ApriltagFamilies.TAG36H11
TAG_SIZE_MM = 30.0

# 相机内参
FX = -1.0
FY = -1.0
CX = CAM_WIDTH // 2
CY = CAM_HEIGHT // 2

# 手眼偏移
OFFSET_X = 0.0
OFFSET_Y = 0.0
OFFSET_Z = 50.0

# 精定位配置
ALIGN_CENTER_X = CAM_WIDTH // 2   # 画面中心X（夹爪对准位置）
ALIGN_CENTER_Y = CAM_HEIGHT // 2  # 画面中心Y
ALIGN_STABLE_FRAMES = 2           # 精定位需要连续检测帧数

# 稳定性过滤
STABLE_THRESHOLD = 3
tag_detect_count = {}
tag_lost_count = {}

# 工作模式
MODE_IDLE = 0
MODE_SCAN = 1
MODE_ALIGN = 2

current_mode = MODE_IDLE
align_target_id = -1


# ========== UART初始化 ==========
def init_uart():
    device_id = sys.device_id()

    if device_id == "maixcam2":
        pin_function = {"A21": "UART4_TX", "A22": "UART4_RX"}
        device = "/dev/ttyS4"
    else:
        pin_function = {"A16": "UART0_TX", "A17": "UART0_RX"}
        device = "/dev/ttyS0"

    for pin, func in pin_function.items():
        err.check_raise(pinmap.set_pin_function(pin, func),
                        "Failed set pin {} to {}".format(pin, func))

    serial = uart.UART(device, 115200)
    print("[UART] {} @ 115200".format(device))
    return serial


# ========== 坐标转换 ==========
def camera_to_base(tx, ty, tz):
    cam_x = tx * TAG_SIZE_MM
    cam_y = ty * TAG_SIZE_MM
    cam_z = tz * TAG_SIZE_MM

    base_x = cam_z + OFFSET_X
    base_y = -cam_x + OFFSET_Y
    base_z = -cam_y + OFFSET_Z

    return base_x, base_y, base_z


def pixel_to_mm(px, py, distance_mm):
    """
    像素偏差转换为毫米偏差
    px, py: 像素偏差（相对于画面中心）
    distance_mm: 当前距离（用于计算比例）
    """
    # 简化计算：假设30mm的Tag在特定距离下占据特定像素
    # 实际应该用相机内参计算，这里用经验值
    # 距离越远，同样的像素偏差对应更大的实际偏差

    # 经验公式：1像素 约等于 distance_mm / 500 毫米
    scale = abs(distance_mm) / 500.0 if distance_mm != 0 else 0.5

    dx_mm = px * scale
    dy_mm = py * scale

    return dx_mm, dy_mm


# ========== 命令解析 ==========
def parse_command(cmd):
    """解析RA6M5发来的命令"""
    global current_mode, align_target_id

    cmd = cmd.strip()

    if cmd == "SCAN":
        current_mode = MODE_SCAN
        return True

    if cmd.startswith("ALIGN:"):
        try:
            align_target_id = int(cmd[6:])
            current_mode = MODE_ALIGN
            return True
        except:
            return False

    return False


# ========== 主程序 ==========
def main():
    global current_mode, align_target_id
    global tag_detect_count, tag_lost_count

    print("=" * 40)
    print("  MaixCam2 Visual Servo")
    print("  Device: {}".format(sys.device_id()))
    print("=" * 40)

    cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT)
    disp = display.Display()
    serial = init_uart()

    uart_buffer = ""
    align_stable_count = 0
    last_align_dx = 0
    last_align_dy = 0

    print("\n[Ready] Waiting for commands...\n")

    while not app.need_exit():
        # 读取UART命令
        if serial.available():
            data = serial.read_str()
            if data:
                uart_buffer += data
                while "\n" in uart_buffer:
                    line, uart_buffer = uart_buffer.split("\n", 1)
                    if line:
                        print("[CMD] {}".format(line))
                        parse_command(line)

        # 拍照
        img = cam.read()

        # 检测AprilTag
        apriltags = img.find_apriltags(families=FAMILIES)

        # 当前帧检测到的ID
        detected_ids = set()

        # ========== 扫描模式 ==========
        if current_mode == MODE_SCAN:
            serial.write_str("SCAN_START\n")

            # 等待几帧让检测稳定
            for _ in range(5):
                img = cam.read()
                apriltags = img.find_apriltags(families=FAMILIES)
                time.sleep_ms(50)

            # 发送所有检测到的Tag
            for a in apriltags:
                tx = a.x_translation()
                ty = a.y_translation()
                tz = a.z_translation()
                base_x, base_y, base_z = camera_to_base(tx, ty, tz)

                msg = "TAG:{},{:.1f},{:.1f},{:.1f}\n".format(
                    a.id(), base_x, base_y, base_z)
                serial.write_str(msg)
                print("[SCAN] {}".format(msg.strip()))

            serial.write_str("SCAN_END\n")
            current_mode = MODE_IDLE
            print("[SCAN] Done, {} tags found".format(len(apriltags)))

        # ========== 精定位模式 ==========
        elif current_mode == MODE_ALIGN:
            target_found = False

            for a in apriltags:
                if a.id() == align_target_id:
                    target_found = True

                    # 计算像素偏差（相对于画面中心）
                    px = a.cx() - ALIGN_CENTER_X
                    py = a.cy() - ALIGN_CENTER_Y

                    # 获取距离
                    tz = a.z_translation()
                    distance_mm = tz * TAG_SIZE_MM

                    # 转换为毫米偏差
                    dx_mm, dy_mm = pixel_to_mm(px, py, distance_mm)

                    # 画十字准星
                    img.draw_line(ALIGN_CENTER_X - 20, ALIGN_CENTER_Y,
                                  ALIGN_CENTER_X + 20, ALIGN_CENTER_Y,
                                  image.COLOR_RED, 2)
                    img.draw_line(ALIGN_CENTER_X, ALIGN_CENTER_Y - 20,
                                  ALIGN_CENTER_X, ALIGN_CENTER_Y + 20,
                                  image.COLOR_RED, 2)

                    # 画目标位置
                    img.draw_circle(a.cx(), a.cy(), 10, image.COLOR_GREEN, 2)

                    # 显示偏差
                    img.draw_string(10, 10,
                        "dx:{:.1f} dy:{:.1f}".format(dx_mm, dy_mm),
                        image.COLOR_WHITE)

                    # 检查稳定性
                    if (abs(dx_mm - last_align_dx) < 3 and
                        abs(dy_mm - last_align_dy) < 3):
                        align_stable_count += 1
                    else:
                        align_stable_count = 0

                    last_align_dx = dx_mm
                    last_align_dy = dy_mm

                    # 稳定后发送结果
                    if align_stable_count >= ALIGN_STABLE_FRAMES:
                        # 坐标系转换：像素Y向下为正，机械臂Y可能不同
                        # 这里假设：像素X+ = 机械臂Y+，像素Y+ = 机械臂X-
                        # 根据实际安装调整
                        arm_dx = -dy_mm  # 像素Y -> 机械臂X
                        arm_dy = dx_mm   # 像素X -> 机械臂Y

                        msg = "ALIGN_OK:{:.1f},{:.1f}\n".format(arm_dx, arm_dy)
                        serial.write_str(msg)
                        print("[ALIGN] {}".format(msg.strip()))

                        current_mode = MODE_IDLE
                        align_stable_count = 0

                    break

            if not target_found:
                # 目标丢失
                align_stable_count = 0
                img.draw_string(10, 10, "Target {} not found".format(align_target_id),
                               image.COLOR_RED)

                # 超时处理（这里简化为立即返回失败）
                serial.write_str("ALIGN_FAIL\n")
                print("[ALIGN] Target {} not found".format(align_target_id))
                current_mode = MODE_IDLE

        # ========== 空闲模式 - 持续显示 ==========
        else:
            for a in apriltags:
                # 画框
                corners = a.corners()
                for i in range(4):
                    img.draw_line(corners[i][0], corners[i][1],
                                  corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                                  image.COLOR_GREEN, 2)

                # 显示ID
                img.draw_string(a.cx() - 10, a.cy() - 30,
                               "ID:{}".format(a.id()), image.COLOR_GREEN)

        # 显示模式
        mode_str = ["IDLE", "SCAN", "ALIGN"][current_mode]
        img.draw_string(CAM_WIDTH - 60, 10, mode_str, image.COLOR_YELLOW)

        disp.show(img)
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
