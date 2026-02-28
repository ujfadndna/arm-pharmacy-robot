"""
vision_udp.py - MaixCam2 WiFi UDP通信模块
与RA6M5通过W800进行UDP通信，发送视觉识别结果

协议格式:
| 帧头      | CMD  | LEN    | DATA    | CHK   |
| 0xAA 0x55 | 1B   | 2B(LE) | N字节   | 1B    |

使用方法:
    from vision_udp import VisionUDP

    udp = VisionUDP(target_ip="192.168.4.1", target_port=8888)
    udp.connect()

    # 发送检测结果
    udp.send_detect_result(objs)

    # 发送QR码结果
    udp.send_qr_result("DRUG_001")
"""

import socket
import struct
import threading
import time

# 协议常量
FRAME_HEADER = bytes([0xAA, 0x55])

CMD_HEARTBEAT = 0x01
CMD_DETECT_RESULT = 0x02
CMD_QR_RESULT = 0x03
CMD_TASK_REQUEST = 0x10
CMD_TASK_ACK = 0x11

# 任务类型
TASK_SCAN_QR = 0x01
TASK_DETECT_DRUG = 0x02
TASK_CALIBRATE = 0x03


class VisionUDP:
    """MaixCam2 UDP通信类"""

    def __init__(self, target_ip: str, target_port: int = 8888,
                 local_port: int = 8889):
        """
        初始化UDP通信

        Args:
            target_ip: RA6M5/W800的IP地址
            target_port: 目标UDP端口
            local_port: 本地监听端口
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.local_port = local_port

        self.socket = None
        self.running = False
        self.rx_thread = None

        # 回调函数
        self.on_task_request = None
        self.on_heartbeat = None

    def connect(self) -> bool:
        """建立UDP连接"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(('0.0.0.0', self.local_port))
            self.socket.settimeout(0.1)

            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()

            print(f"[VisionUDP] Listening on port {self.local_port}")
            print(f"[VisionUDP] Target: {self.target_ip}:{self.target_port}")
            return True
        except Exception as e:
            print(f"[VisionUDP] Connect failed: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.socket:
            self.socket.close()
            self.socket = None

    def _calc_checksum(self, cmd: int, data: bytes) -> int:
        """计算校验和"""
        chk = cmd
        length = len(data)
        chk ^= (length & 0xFF)
        chk ^= (length >> 8) & 0xFF
        for b in data:
            chk ^= b
        return chk

    def _build_frame(self, cmd: int, data: bytes = b'') -> bytes:
        """构建协议帧"""
        length = len(data)
        chk = self._calc_checksum(cmd, data)
        frame = FRAME_HEADER + bytes([cmd])
        frame += struct.pack('<H', length)
        frame += data
        frame += bytes([chk])
        return frame

    def _send(self, cmd: int, data: bytes = b'') -> bool:
        """发送数据帧"""
        if not self.socket:
            return False
        try:
            frame = self._build_frame(cmd, data)
            self.socket.sendto(frame, (self.target_ip, self.target_port))
            return True
        except Exception as e:
            print(f"[VisionUDP] Send failed: {e}")
            return False

    def _rx_loop(self):
        """接收线程"""
        buffer = b''
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                buffer += data
                buffer = self._parse_buffer(buffer)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[VisionUDP] RX error: {e}")

    def _parse_buffer(self, buffer: bytes) -> bytes:
        """解析接收缓冲区"""
        while len(buffer) >= 6:  # 最小帧长度
            # 查找帧头
            idx = buffer.find(FRAME_HEADER)
            if idx < 0:
                return b''
            if idx > 0:
                buffer = buffer[idx:]

            if len(buffer) < 6:
                break

            cmd = buffer[2]
            length = struct.unpack('<H', buffer[3:5])[0]
            frame_len = 6 + length

            if len(buffer) < frame_len:
                break

            data = buffer[5:5+length]
            chk = buffer[5+length]

            # 校验
            if chk == self._calc_checksum(cmd, data):
                self._process_frame(cmd, data)

            buffer = buffer[frame_len:]

        return buffer

    def _process_frame(self, cmd: int, data: bytes):
        """处理接收到的帧"""
        if cmd == CMD_HEARTBEAT:
            if self.on_heartbeat:
                self.on_heartbeat()
        elif cmd == CMD_TASK_REQUEST:
            if len(data) >= 1 and self.on_task_request:
                task_type = data[0]
                params = data[1:] if len(data) > 1 else b''
                self.on_task_request(task_type, params)

    # ========== 公共发送接口 ==========

    def send_heartbeat(self) -> bool:
        """发送心跳"""
        return self._send(CMD_HEARTBEAT)

    def send_detect_result(self, objs: list) -> bool:
        """
        发送检测结果

        Args:
            objs: 检测对象列表，每个对象需要有 x, y, w, h, class_id, score 属性
        """
        if not objs:
            return self._send(CMD_DETECT_RESULT, bytes([0]))

        count = min(len(objs), 16)
        data = bytes([count])

        for i in range(count):
            obj = objs[i]
            # 兼容MaixPy的检测结果对象
            x = getattr(obj, 'x', obj.get('x', 0) if isinstance(obj, dict) else 0)
            y = getattr(obj, 'y', obj.get('y', 0) if isinstance(obj, dict) else 0)
            w = getattr(obj, 'w', obj.get('w', 0) if isinstance(obj, dict) else 0)
            h = getattr(obj, 'h', obj.get('h', 0) if isinstance(obj, dict) else 0)
            class_id = getattr(obj, 'class_id',
                              obj.get('class_id', 0) if isinstance(obj, dict) else 0)
            score = getattr(obj, 'score',
                           obj.get('score', 0.0) if isinstance(obj, dict) else 0.0)

            # 打包: 2B x + 2B y + 2B w + 2B h + 2B class_id + 4B score
            data += struct.pack('<hhHHHf', int(x), int(y), int(w), int(h),
                               int(class_id), float(score))

        return self._send(CMD_DETECT_RESULT, data)

    def send_qr_result(self, qr_data: str) -> bool:
        """发送QR码扫描结果"""
        encoded = qr_data.encode('utf-8')[:128]
        data = bytes([len(encoded)]) + encoded
        return self._send(CMD_QR_RESULT, data)

    def send_task_ack(self, task_type: int, success: bool = True) -> bool:
        """发送任务确认"""
        data = bytes([task_type, 0x01 if success else 0x00])
        return self._send(CMD_TASK_ACK, data)
