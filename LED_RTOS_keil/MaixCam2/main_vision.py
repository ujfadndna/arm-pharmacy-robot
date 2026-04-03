"""
main_vision.py - MaixCam2 视觉识别主程序
集成YOLO检测 + UDP通信 + HTTP视频流

功能:
1. YOLO药品检测
2. UDP发送检测结果给RA6M5
3. HTTP视频流供评委观看

使用前配置:
- WIFI_SSID: WiFi名称
- WIFI_PASSWORD: WiFi密码
- TARGET_IP: RA6M5/W800的IP地址
"""

from maix import camera, display, image, nn, app
from maix import network, err
import time

from vision_udp import VisionUDP, TASK_SCAN_QR, TASK_DETECT_DRUG

# ========== 配置 ==========
WIFI_SSID = "yamaRedmi K70"
WIFI_PASSWORD = "12345678"
TARGET_IP = "192.168.4.1"  # W800的IP地址
TARGET_PORT = 8888

# ========== 全局变量 ==========
udp = None
detector = None
cam = None
disp = None

def connect_wifi():
    """连接WiFi"""
    print("[WiFi] Connecting...")
    w = network.wifi.Wifi()
    e = w.connect(WIFI_SSID, WIFI_PASSWORD, wait=True, timeout=30)
    if e != err.Err.ERR_NONE:
        print(f"[WiFi] Connect failed: {e}")
        return None
    print(f"[WiFi] Connected, IP: {w.get_ip()}")
    return w

def on_task_request(task_type: int, params: bytes):
    """处理RA6M5发来的任务请求"""
    print(f"[Task] Received task: type={task_type}, params={params.hex()}")

    if task_type == TASK_SCAN_QR:
        # TODO: 执行QR码扫描
        pass
    elif task_type == TASK_DETECT_DRUG:
        # TODO: 执行药品检测
        pass

    # 发送确认
    if udp:
        udp.send_task_ack(task_type, True)

def main():
    global udp, detector, cam, disp

    # 1. 连接WiFi
    wifi = connect_wifi()
    if not wifi:
        print("[Error] WiFi connection failed, exiting...")
        return

    # 2. 初始化UDP通信
    udp = VisionUDP(target_ip=TARGET_IP, target_port=TARGET_PORT)
    udp.on_task_request = on_task_request
    if not udp.connect():
        print("[Error] UDP connection failed")
        return

    # 3. 初始化YOLO检测器
    print("[YOLO] Loading model...")
    detector = nn.YOLOv5(model="/root/models/yolov5s.mud")
    print(f"[YOLO] Model loaded, input: {detector.input_width()}x{detector.input_height()}")

    # 4. 初始化摄像头和显示
    cam = camera.Camera(detector.input_width(), detector.input_height(),
                        detector.input_format())
    disp = display.Display()

    print("[Main] Starting main loop...")
    last_heartbeat = time.time()
    frame_count = 0

    # 5. 主循环
    while not app.need_exit():
        # 读取图像
        img = cam.read()

        # YOLO检测
        objs = detector.detect(img, conf_th=0.5, iou_th=0.45)

        # 发送检测结果
        if len(objs) > 0:
            udp.send_detect_result(objs)
            print(f"[Detect] Found {len(objs)} objects")

        # 绘制检测框
        for obj in objs:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
            msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
            img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)

        # 显示帧率
        frame_count += 1
        if frame_count % 30 == 0:
            img.draw_string(10, 10, f"FPS: {30}", color=image.COLOR_GREEN)

        # 显示图像
        disp.show(img)

        # 定时发送心跳
        now = time.time()
        if now - last_heartbeat > 1.0:
            udp.send_heartbeat()
            last_heartbeat = now

    # 清理
    udp.disconnect()
    print("[Main] Exited")


if __name__ == "__main__":
    main()
