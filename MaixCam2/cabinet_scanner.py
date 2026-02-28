"""
cabinet_scanner.py - 药柜AprilTag扫描器

功能：
1. 扫描药柜中的所有AprilTag
2. 获取每个药盒的6D位姿
3. 坐标转换（相机→基座）
4. 通过UART发送给RA6M5

通信协议：
  SCAN_START\n
  TAG:<id>,<x>,<y>,<z>\n
  TAG:<id>,<x>,<y>,<z>\n
  ...
  SCAN_END\n
"""

from maix import camera, display, image, uart
import time
import math

# ========== 配置 ==========
CAM_WIDTH = 640
CAM_HEIGHT = 480
APRILTAG_FAMILY = image.ApriltagFamilies.TAG36H11
APRILTAG_SIZE_MM = 30.0  # AprilTag实际尺寸 (mm)

# UART配置
UART_PORT = "/dev/ttyS1"  # MaixCam2的UART1
UART_BAUDRATE = 115200

# ========== 手眼标定参数 ==========
# 相机内参 (需要标定后填入)
CAMERA_FX = 600.0  # 焦距x
CAMERA_FY = 600.0  # 焦距y
CAMERA_CX = 320.0  # 主点x
CAMERA_CY = 240.0  # 主点y

# 手眼变换矩阵 (相机坐标系 → 基座坐标系)
# 需要通过手眼标定获取，这里使用简化的偏移量
HAND_EYE_OFFSET = {
    'x': 0.0,    # X偏移 (mm)
    'y': 0.0,    # Y偏移 (mm)
    'z': 50.0,   # Z偏移 (mm) - 相机到夹爪的距离
}

# 坐标系旋转 (简化版，假设相机朝下)
# 实际使用时需要根据安装方式调整
def camera_to_base(cam_x, cam_y, cam_z):
    """
    相机坐标系 → 基座坐标系

    相机坐标系: X右, Y下, Z前
    基座坐标系: X前, Y左, Z上
    """
    base_x = cam_z + HAND_EYE_OFFSET['x']
    base_y = -cam_x + HAND_EYE_OFFSET['y']
    base_z = -cam_y + HAND_EYE_OFFSET['z']
    return base_x, base_y, base_z


# ========== UART通信 ==========
class UARTComm:
    def __init__(self, port=UART_PORT, baudrate=UART_BAUDRATE):
        self.uart = uart.UART(port, baudrate)
        print(f"[UART] Opened {port} @ {baudrate}")

    def send(self, msg):
        """发送消息"""
        if not msg.endswith('\n'):
            msg += '\n'
        self.uart.write(msg.encode('utf-8'))
        print(f"[UART TX] {msg.strip()}")

    def send_scan_start(self):
        self.send("SCAN_START")

    def send_scan_end(self):
        self.send("SCAN_END")

    def send_tag(self, tag_id, x, y, z):
        """发送单个TAG检测结果"""
        msg = f"TAG:{tag_id},{x:.1f},{y:.1f},{z:.1f}"
        self.send(msg)


# ========== AprilTag检测 ==========
class CabinetScanner:
    def __init__(self):
        self.cam = camera.Camera(CAM_WIDTH, CAM_HEIGHT)
        self.disp = display.Display()
        self.uart_comm = UARTComm()
        print("[Scanner] Initialized")

    def detect_tags(self, img):
        """检测图像中的所有AprilTag"""
        tags = img.find_apriltags(
            families=APRILTAG_FAMILY,
            fx=CAMERA_FX,
            fy=CAMERA_FY,
            cx=CAMERA_CX,
            cy=CAMERA_CY
        )
        return tags

    def process_tag(self, tag):
        """处理单个AprilTag，返回基座坐标"""
        tag_id = tag.id()

        # 获取相机坐标系下的位置 (单位: AprilTag尺寸)
        # 需要乘以实际尺寸转换为mm
        cam_x = tag.x_translation() * APRILTAG_SIZE_MM
        cam_y = tag.y_translation() * APRILTAG_SIZE_MM
        cam_z = tag.z_translation() * APRILTAG_SIZE_MM

        # 转换到基座坐标系
        base_x, base_y, base_z = camera_to_base(cam_x, cam_y, cam_z)

        return {
            'id': tag_id,
            'cam': (cam_x, cam_y, cam_z),
            'base': (base_x, base_y, base_z),
            'corners': tag.corners(),
            'cx': tag.cx(),
            'cy': tag.cy()
        }

    def scan_cabinet(self):
        """
        扫描药柜，返回所有检测到的药品位置
        """
        print("\n[Scanner] Starting cabinet scan...")

        # 多帧检测，取稳定结果
        all_results = {}
        SCAN_FRAMES = 10

        for frame_idx in range(SCAN_FRAMES):
            img = self.cam.read()
            tags = self.detect_tags(img)

            for tag in tags:
                result = self.process_tag(tag)
                tag_id = result['id']

                if tag_id not in all_results:
                    all_results[tag_id] = []
                all_results[tag_id].append(result)

            # 显示检测结果
            for tag in tags:
                corners = tag.corners()
                for i in range(4):
                    img.draw_line(
                        corners[i][0], corners[i][1],
                        corners[(i+1)%4][0], corners[(i+1)%4][1],
                        image.COLOR_GREEN, 2
                    )
                img.draw_string(
                    tag.cx() - 20, tag.cy() - 10,
                    f"ID:{tag.id()}",
                    image.COLOR_RED
                )

            self.disp.show(img)
            time.sleep(0.05)

        # 计算平均位置
        final_results = []
        for tag_id, detections in all_results.items():
            if len(detections) >= SCAN_FRAMES // 2:  # 至少检测到一半帧数
                avg_x = sum(d['base'][0] for d in detections) / len(detections)
                avg_y = sum(d['base'][1] for d in detections) / len(detections)
                avg_z = sum(d['base'][2] for d in detections) / len(detections)
                final_results.append({
                    'id': tag_id,
                    'x': avg_x,
                    'y': avg_y,
                    'z': avg_z,
                    'count': len(detections)
                })

        print(f"[Scanner] Found {len(final_results)} tags")
        return final_results

    def send_results(self, results):
        """通过UART发送扫描结果给RA6M5"""
        self.uart_comm.send_scan_start()

        for r in results:
            self.uart_comm.send_tag(r['id'], r['x'], r['y'], r['z'])

        self.uart_comm.send_scan_end()
        print(f"[Scanner] Sent {len(results)} tags to RA6M5")

    def run_once(self):
        """执行一次扫描并发送结果"""
        results = self.scan_cabinet()
        if results:
            self.send_results(results)
        return results

    def run_continuous(self, interval_sec=5.0):
        """持续扫描模式"""
        print(f"[Scanner] Continuous mode, interval={interval_sec}s")
        print("[Scanner] Press Ctrl+C to stop")

        while True:
            try:
                self.run_once()
                time.sleep(interval_sec)
            except KeyboardInterrupt:
                print("\n[Scanner] Stopped")
                break

    def run_preview(self):
        """预览模式 - 只显示检测结果，不发送"""
        print("[Scanner] Preview mode - Press Ctrl+C to stop")

        while True:
            try:
                img = self.cam.read()
                tags = self.detect_tags(img)

                for tag in tags:
                    result = self.process_tag(tag)

                    # 画框
                    corners = tag.corners()
                    for i in range(4):
                        img.draw_line(
                            corners[i][0], corners[i][1],
                            corners[(i+1)%4][0], corners[(i+1)%4][1],
                            image.COLOR_GREEN, 2
                        )

                    # 显示信息
                    x, y, z = result['base']
                    info1 = f"ID:{result['id']}"
                    info2 = f"({x:.0f},{y:.0f},{z:.0f})"
                    img.draw_string(tag.cx() - 30, tag.cy() - 25, info1, image.COLOR_RED)
                    img.draw_string(tag.cx() - 40, tag.cy() + 10, info2, image.COLOR_BLUE)

                # 显示检测数量
                img.draw_string(10, 10, f"Tags: {len(tags)}", image.COLOR_WHITE)

                self.disp.show(img)
                time.sleep(0.05)

            except KeyboardInterrupt:
                print("\n[Scanner] Stopped")
                break


# ========== 主程序 ==========
def main():
    import sys

    print("=" * 50)
    print("  药柜AprilTag扫描器")
    print("=" * 50)
    print()
    print("用法:")
    print("  python cabinet_scanner.py          # 单次扫描")
    print("  python cabinet_scanner.py preview  # 预览模式")
    print("  python cabinet_scanner.py loop     # 持续扫描")
    print()

    scanner = CabinetScanner()

    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        if mode == 'preview':
            scanner.run_preview()
        elif mode == 'loop':
            scanner.run_continuous()
        else:
            print(f"未知模式: {mode}")
    else:
        # 默认单次扫描
        results = scanner.run_once()
        print("\n扫描结果:")
        for r in results:
            print(f"  ID={r['id']}: ({r['x']:.1f}, {r['y']:.1f}, {r['z']:.1f}) mm")


if __name__ == "__main__":
    main()
