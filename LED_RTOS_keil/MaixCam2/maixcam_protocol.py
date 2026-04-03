"""
maixcam_protocol.py - MaixCam2 <-> RA6M5 UART 二进制协议
"""

import struct

try:
    import machine
except ImportError:
    machine = None

try:
    import serial
except ImportError:
    serial = None


class _DummyUart:
    """无串口环境下的占位实现，方便本地调试打包结果。"""

    def __init__(self):
        self.tx_frames = []

    def write(self, data):
        self.tx_frames.append(bytes(data))
        return len(data)

    def read(self, _n=1):
        return b""

    def any(self):
        return 0

    @property
    def in_waiting(self):
        return 0


class MaixCamProtocol:
    FRAME_HEADER = bytes([0xAA, 0x55])
    FRAME_MAX_DATA = 256

    # MaixCam2 -> RA6M5
    CMD_HEARTBEAT = 0x01
    CMD_TAG_SINGLE = 0x02
    CMD_SCAN_START = 0x03
    CMD_SCAN_TAG = 0x04
    CMD_SCAN_END = 0x05
    CMD_ALIGN_OK = 0x06
    CMD_ALIGN_FAIL = 0x07
    CMD_QR_RESULT = 0x08
    CMD_YOLO_RESULT = 0x09
    CMD_VOICE_ACTION = 0x0A
    CMD_ACK = 0x0B

    # RA6M5 -> MaixCam2
    CMD_REQ_SCAN = 0x20
    CMD_REQ_ALIGN = 0x21
    CMD_REQ_QR = 0x22
    CMD_REQ_YOLO = 0x23
    CMD_SPEAK = 0x24
    CMD_SET_MODE = 0x25

    _ST_HEADER_0 = 0
    _ST_HEADER_1 = 1
    _ST_CMD = 2
    _ST_LEN_L = 3
    _ST_LEN_H = 4
    _ST_DATA = 5
    _ST_CHK = 6

    def __init__(self, uart_dev="/dev/ttyS1", baudrate=115200):
        self.uart = self._init_uart(uart_dev, baudrate)
        self.last_tx_frame = b""

        self._state = self._ST_HEADER_0
        self._frame_cmd = 0
        self._frame_len = 0
        self._frame_idx = 0
        self._frame_data = bytearray()
        self._frame_chk = 0

        # 由用户赋值
        self.on_req_scan = None
        self.on_req_align = None
        self.on_req_qr = None
        self.on_req_yolo = None
        self.on_speak = None
        self.on_set_mode = None

    def _init_uart(self, uart_dev, baudrate):
        if machine is not None and hasattr(machine, "UART"):
            try:
                return machine.UART(uart_dev, baudrate=baudrate)
            except Exception:
                try:
                    return machine.UART(1, baudrate=baudrate)
                except Exception:
                    pass

        if serial is not None:
            try:
                return serial.Serial(uart_dev, baudrate=baudrate, timeout=0)
            except Exception:
                pass

        return _DummyUart()

    def _calc_chk(self, cmd, data):
        chk = cmd & 0xFF
        data_len = len(data) & 0xFFFF
        chk ^= (data_len & 0xFF)
        chk ^= ((data_len >> 8) & 0xFF)
        for b in data:
            chk ^= b
        return chk & 0xFF

    def send(self, cmd, data=b""):
        if data is None:
            data = b""
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("data must be bytes or bytearray")
        if len(data) > self.FRAME_MAX_DATA:
            raise ValueError("data length exceeds FRAME_MAX_DATA")

        data = bytes(data)
        frame = bytearray(self.FRAME_HEADER)
        frame.append(cmd & 0xFF)
        frame.append(len(data) & 0xFF)
        frame.append((len(data) >> 8) & 0xFF)
        frame.extend(data)
        frame.append(self._calc_chk(cmd, data))

        self.last_tx_frame = bytes(frame)
        self.uart.write(self.last_tx_frame)
        return self.last_tx_frame

    def poll(self):
        handled = 0
        while True:
            n = self._uart_available()
            if n <= 0:
                break

            raw = self.uart.read(n)
            if not raw:
                break

            for b in raw:
                if self._feed_byte(b):
                    handled += 1
        return handled

    def _uart_available(self):
        if hasattr(self.uart, "any"):
            try:
                n = self.uart.any()
                return int(n) if n else 0
            except Exception:
                pass

        if hasattr(self.uart, "in_waiting"):
            try:
                return int(self.uart.in_waiting)
            except Exception:
                pass
        return 0

    def _feed_byte(self, b):
        if isinstance(b, bytes):
            b = b[0]
        b &= 0xFF

        if self._state == self._ST_HEADER_0:
            if b == self.FRAME_HEADER[0]:
                self._state = self._ST_HEADER_1
            return False

        if self._state == self._ST_HEADER_1:
            if b == self.FRAME_HEADER[1]:
                self._state = self._ST_CMD
            else:
                self._state = self._ST_HEADER_0
            return False

        if self._state == self._ST_CMD:
            self._frame_cmd = b
            self._frame_chk = b
            self._state = self._ST_LEN_L
            return False

        if self._state == self._ST_LEN_L:
            self._frame_len = b
            self._frame_chk ^= b
            self._state = self._ST_LEN_H
            return False

        if self._state == self._ST_LEN_H:
            self._frame_len |= (b << 8)
            self._frame_chk ^= b
            self._frame_idx = 0
            self._frame_data = bytearray()
            if self._frame_len == 0:
                self._state = self._ST_CHK
            elif self._frame_len <= self.FRAME_MAX_DATA:
                self._state = self._ST_DATA
            else:
                self._state = self._ST_HEADER_0
            return False

        if self._state == self._ST_DATA:
            self._frame_data.append(b)
            self._frame_chk ^= b
            self._frame_idx += 1
            if self._frame_idx >= self._frame_len:
                self._state = self._ST_CHK
            return False

        if self._state == self._ST_CHK:
            ok = (b == (self._frame_chk & 0xFF))
            if ok:
                self._dispatch(self._frame_cmd, bytes(self._frame_data))
            self._state = self._ST_HEADER_0
            return ok

        self._state = self._ST_HEADER_0
        return False

    def _dispatch(self, cmd, data):
        if cmd == self.CMD_REQ_SCAN:
            if self.on_req_scan and len(data) >= 2:
                scan_type, frames_req = struct.unpack("<BB", data[:2])
                self.on_req_scan(scan_type, frames_req)
            return

        if cmd == self.CMD_REQ_ALIGN:
            if self.on_req_align and len(data) >= 4:
                target_id, timeout_ms, stable_req = struct.unpack("<BHB", data[:4])
                self.on_req_align(target_id, timeout_ms, stable_req)
            return

        if cmd == self.CMD_REQ_QR:
            if self.on_req_qr and len(data) >= 2:
                (timeout_ms,) = struct.unpack("<H", data[:2])
                self.on_req_qr(timeout_ms)
            return

        if cmd == self.CMD_REQ_YOLO:
            if self.on_req_yolo and len(data) >= 4:
                class_filter, conf_thresh, timeout_ms = struct.unpack("<BBH", data[:4])
                self.on_req_yolo(class_filter, conf_thresh, timeout_ms)
            return

        if cmd == self.CMD_SPEAK:
            if self.on_speak and len(data) >= 1:
                text_len = data[0]
                if len(data) >= (1 + text_len):
                    text = data[1:1 + text_len].decode("utf-8", errors="ignore")
                    self.on_speak(text)
            return

        if cmd == self.CMD_SET_MODE:
            if self.on_set_mode and len(data) >= 2:
                mode, param = struct.unpack("<BB", data[:2])
                self.on_set_mode(mode, param)
            return

    # ========== 发送快捷函数 ==========
    def send_tag_single(self, tag_id, x, y, z, conf):
        data = struct.pack("<BfffB", int(tag_id) & 0xFF, float(x), float(y), float(z), int(conf) & 0xFF)
        return self.send(self.CMD_TAG_SINGLE, data)

    def send_scan_start(self):
        return self.send(self.CMD_SCAN_START)

    def send_scan_tag(self, tag_id, x, y, z, conf):
        data = struct.pack("<BfffB", int(tag_id) & 0xFF, float(x), float(y), float(z), int(conf) & 0xFF)
        return self.send(self.CMD_SCAN_TAG, data)

    def send_scan_end(self, total_count):
        data = struct.pack("<B", int(total_count) & 0xFF)
        return self.send(self.CMD_SCAN_END, data)

    def send_align_ok(self, target_id, dx, dy, stable):
        data = struct.pack("<BffB", int(target_id) & 0xFF, float(dx), float(dy), int(stable) & 0xFF)
        return self.send(self.CMD_ALIGN_OK, data)

    def send_align_fail(self, target_id, reason):
        data = struct.pack("<BB", int(target_id) & 0xFF, int(reason) & 0xFF)
        return self.send(self.CMD_ALIGN_FAIL, data)

    def send_qr_result(self, data_str):
        if isinstance(data_str, str):
            payload = data_str.encode("utf-8")
        else:
            payload = bytes(data_str)
        if len(payload) > 255:
            payload = payload[:255]
        data = bytes([len(payload)]) + payload
        return self.send(self.CMD_QR_RESULT, data)

    def send_yolo_result(self, objects):
        # objects: [(x, y, w, h, class_id, score), ...]
        count = min(len(objects), 16)
        data = bytearray([count])
        for i in range(count):
            x, y, w, h, class_id, score = objects[i]
            data.extend(struct.pack(
                "<hhHHHf",
                int(x),
                int(y),
                int(w) & 0xFFFF,
                int(h) & 0xFFFF,
                int(class_id) & 0xFFFF,
                float(score),
            ))
        return self.send(self.CMD_YOLO_RESULT, data)

    def send_voice_action(self, action_type, slot_from, slot_to):
        from_bytes = str(slot_from).encode("ascii", errors="ignore")[:3].ljust(3, b"\x00")
        to_bytes = str(slot_to).encode("ascii", errors="ignore")[:3].ljust(3, b"\x00")
        data = struct.pack("<B3s3sB", int(action_type) & 0xFF, from_bytes, to_bytes, 0)
        return self.send(self.CMD_VOICE_ACTION, data)


if __name__ == "__main__":
    proto = MaixCamProtocol()
    frame = proto.send_tag_single(tag_id=1, x=100.0, y=200.0, z=-50.0, conf=90)
    print("TAG_SINGLE frame:", " ".join("{:02X}".format(b) for b in frame))
