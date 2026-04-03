#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tts_server.py - MaixCam2 HTTP TTS 播报服务（讯飞云端语音合成）

在 MaixCam2 上启动 HTTP 服务（端口 8000），接收文本，调用讯飞 TTS 合成语音并播放。

用法:
  POST /speak  body=UTF-8文本
  GET  /speak?text=你好

电脑端调用:
  requests.post("http://<IP>:8000/speak", data="正在抓取阿莫西林".encode())
  或浏览器: http://<IP>:8000/speak?text=正在抓取阿莫西林
"""

import socket
import ssl
import os
import time
import json
import base64
import hmac
import hashlib
import struct
import threading
from queue import Queue, Empty
from datetime import datetime, timezone
from urllib.parse import urlencode, urlparse

# ========== 平台检测 ==========
try:
    from maix import audio, camera, display, app
    MAIX_AVAILABLE = True
except ImportError:
    MAIX_AVAILABLE = False
    print("[WARN] maix not available, run as PC mock")

from flask import Flask, request as flask_request

# ========== 配置 ==========
XFYUN_APP_ID     = "98d6a094"
XFYUN_API_KEY    = "02f3d11253c7d03fc036b7ddc2f5da07"
XFYUN_API_SECRET = "MWQ1ZjhjYjMwOGVmZWYyNzk1MGUxZDNm"

# 讯飞 TTS 配置
XFYUN_TTS_HOST = "tts-api.xfyun.cn"
XFYUN_TTS_PATH = "/v2/tts"
TTS_VCN = "xiaoyan"       # 发音人: xiaoyan(女), aisjiuxu(男), aisxping(女)
TTS_AUE = "raw"           # PCM格式
TTS_SAMPLE_RATE = 16000   # 讯飞TTS输出16kHz
TTS_SPEED = 50            # 语速 0-100
TTS_VOLUME = 80           # 音量 0-100
TTS_PITCH = 50            # 音调 0-100

HTTP_PORT = 8000
MAX_TEXT_LEN = 128

# ========== SimpleWebSocket (复用voice_llm_chat.py) ==========
class SimpleWebSocket:
    def __init__(self):
        self.sock = None
        self.connected = False

    def connect(self, url, timeout=10):
        try:
            parsed = urlparse(url)
            host = parsed.hostname
            port = parsed.port or (443 if parsed.scheme == "wss" else 80)
            path = parsed.path
            if parsed.query:
                path += "?" + parsed.query

            raw_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            raw_sock.settimeout(timeout)

            if parsed.scheme == "wss":
                context = ssl.create_default_context()
                context.check_hostname = False
                context.verify_mode = ssl.CERT_NONE
                self.sock = context.wrap_socket(raw_sock, server_hostname=host)
            else:
                self.sock = raw_sock

            self.sock.connect((host, port))

            key = base64.b64encode(os.urandom(16)).decode()
            handshake = (
                f"GET {path} HTTP/1.1\r\n"
                f"Host: {host}\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                f"Sec-WebSocket-Version: 13\r\n"
                f"\r\n"
            )
            self.sock.sendall(handshake.encode())

            response = b""
            while b"\r\n\r\n" not in response:
                chunk = self.sock.recv(1024)
                if not chunk:
                    return False
                response += chunk

            if b"101" in response:
                self.connected = True
                return True
            return False
        except Exception as e:
            print(f"[WS] Connect failed: {e}")
            return False

    def send_text(self, text):
        if not self.connected:
            return False
        try:
            frame = self._create_frame(text.encode(), opcode=0x01)
            self.sock.sendall(frame)
            return True
        except:
            self.connected = False
            return False

    def _create_frame(self, data, opcode):
        frame = bytearray()
        frame.append(0x80 | opcode)
        length = len(data)
        mask_key = os.urandom(4)
        if length < 126:
            frame.append(0x80 | length)
        elif length < 65536:
            frame.append(0x80 | 126)
            frame.extend(struct.pack(">H", length))
        else:
            frame.append(0x80 | 127)
            frame.extend(struct.pack(">Q", length))
        frame.extend(mask_key)
        masked = bytearray(len(data))
        for i, b in enumerate(data):
            masked[i] = b ^ mask_key[i % 4]
        frame.extend(masked)
        return bytes(frame)

    def __init_buf(self):
        self._buf = b""

    def recv(self, timeout=5):
        """接收一个完整的 WebSocket 文本帧，支持跨 recv 拼接。"""
        if not self.connected:
            return None
        if not hasattr(self, '_buf'):
            self._buf = b""
        self.sock.settimeout(timeout)

        while True:
            # 尝试从缓冲区解析一个完整帧
            result, consumed = self._try_parse_frame(self._buf)
            if result is not None:
                self._buf = self._buf[consumed:]
                return result
            if consumed > 0:
                # 跳过不可解析的帧（如 binary/ping）
                self._buf = self._buf[consumed:]
                continue

            # 缓冲区不够，继续读
            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    return None
                self._buf += chunk
            except socket.timeout:
                return None
            except Exception as e:
                print(f"[WS] Recv error: {e}")
                return None

    def _try_parse_frame(self, data):
        """尝试解析一个 WebSocket 帧。返回 (payload_str|None, bytes_consumed)。"""
        if len(data) < 2:
            return None, 0

        opcode = data[0] & 0x0F
        masked = (data[1] & 0x80) != 0
        payload_len = data[1] & 0x7F
        offset = 2

        if payload_len == 126:
            if len(data) < 4:
                return None, 0
            payload_len = struct.unpack(">H", data[2:4])[0]
            offset = 4
        elif payload_len == 127:
            if len(data) < 10:
                return None, 0
            payload_len = struct.unpack(">Q", data[2:10])[0]
            offset = 10

        if masked:
            offset += 4  # mask key

        total = offset + payload_len
        if len(data) < total:
            return None, 0  # 数据不完整，等更多数据

        if masked:
            mask_key = data[offset - 4:offset]
            payload = bytearray(data[offset:total])
            for i in range(len(payload)):
                payload[i] ^= mask_key[i % 4]
            payload = bytes(payload)
        else:
            payload = data[offset:total]

        # 只处理文本帧(opcode=1)，跳过 binary/ping/pong/close
        if opcode == 0x01:
            try:
                return payload.decode('utf-8'), total
            except:
                return None, total
        else:
            # 跳过非文本帧
            return None, total

    def close(self):
        self.connected = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None


# ========== 讯飞 TTS ==========
def _create_tts_url():
    """生成讯飞 TTS WebSocket 鉴权 URL（复用ASR同款鉴权）"""
    now = datetime.now(timezone.utc)
    date = now.strftime('%a, %d %b %Y %H:%M:%S GMT')

    signature_origin = f"host: {XFYUN_TTS_HOST}\ndate: {date}\nGET {XFYUN_TTS_PATH} HTTP/1.1"
    signature_sha = hmac.new(
        XFYUN_API_SECRET.encode('utf-8'),
        signature_origin.encode('utf-8'),
        hashlib.sha256
    ).digest()
    signature = base64.b64encode(signature_sha).decode('utf-8')

    authorization_origin = (
        f'api_key="{XFYUN_API_KEY}", '
        f'algorithm="hmac-sha256", '
        f'headers="host date request-line", '
        f'signature="{signature}"'
    )
    authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')

    params = {
        "authorization": authorization,
        "date": date,
        "host": XFYUN_TTS_HOST
    }
    return f"wss://{XFYUN_TTS_HOST}{XFYUN_TTS_PATH}?{urlencode(params)}"


def xfyun_tts_synthesize(text):
    """调用讯飞TTS，返回PCM音频数据(16kHz 16bit mono)"""
    url = _create_tts_url()
    ws = SimpleWebSocket()
    if not ws.connect(url, timeout=10):
        print("[TTS] WebSocket connect failed")
        return None

    # 发送合成请求（一次性发完）
    frame = {
        "common": {"app_id": XFYUN_APP_ID},
        "business": {
            "aue": TTS_AUE,
            "auf": f"audio/L16;rate={TTS_SAMPLE_RATE}",
            "vcn": TTS_VCN,
            "tte": "UTF8",
            "speed": TTS_SPEED,
            "volume": TTS_VOLUME,
            "pitch": TTS_PITCH
        },
        "data": {
            "text": base64.b64encode(text.encode('utf-8')).decode('utf-8'),
            "status": 2
        }
    }
    ws.send_text(json.dumps(frame))

    # 接收音频数据
    pcm_chunks = []
    for _ in range(200):  # 最多等20秒
        msg = ws.recv(timeout=0.1)
        if not msg:
            continue
        try:
            data = json.loads(msg)
            code = data.get("code", 0)
            if code != 0:
                print(f"[TTS] Error code: {code}, msg: {data.get('message', '')}")
                break

            audio_b64 = data.get("data", {}).get("audio")
            if audio_b64:
                pcm_chunks.append(base64.b64decode(audio_b64))

            status = data.get("data", {}).get("status", 0)
            if status == 2:
                break
        except Exception as e:
            print(f"[TTS] Parse error: {e}")

    ws.close()

    if pcm_chunks:
        return b"".join(pcm_chunks)
    return None


# ========== 播放 ==========
_player = None
_tts_queue = Queue(maxsize=16)


def _init_player():
    global _player
    if not MAIX_AVAILABLE:
        return
    _player = audio.Player(sample_rate=TTS_SAMPLE_RATE, block=True)
    _player.volume(TTS_VOLUME)
    print("[Player] Ready (16kHz)")


def _tts_worker():
    """后台线程：取文本 → 讯飞合成 → 播放"""
    while True:
        try:
            text = _tts_queue.get(timeout=1.0)
        except Empty:
            continue

        print(f"[TTS] Speaking: {text}")

        pcm = xfyun_tts_synthesize(text)
        if not pcm:
            print("[TTS] Synthesis failed")
            continue

        print(f"[TTS] Got {len(pcm)} bytes PCM")

        if MAIX_AVAILABLE and _player:
            _player.play(pcm)
        else:
            # PC mock: 按时长估算等待
            duration = len(pcm) / (TTS_SAMPLE_RATE * 2)
            print(f"[TTS] Mock play {duration:.1f}s")
            time.sleep(duration)

        print("[TTS] Done")


# ========== Flask 路由 ==========
flask_app = Flask(__name__)


@flask_app.route("/speak", methods=["GET", "POST"])
def speak():
    print(f"[HTTP] {flask_request.method} /speak content_type={flask_request.content_type} data_len={flask_request.content_length}")
    if flask_request.method == "GET":
        text = flask_request.args.get("text", "")
    else:
        text = flask_request.get_data(as_text=True)

    text = text.strip()
    print(f"[HTTP] text='{text}'")
    if not text:
        return "empty text", 400
    if len(text) > MAX_TEXT_LEN:
        text = text[:MAX_TEXT_LEN]

    if _tts_queue.full():
        return "busy", 429

    _tts_queue.put(text)
    return "ok"


@flask_app.route("/")
def index():
    return "MaixCam2 TTS Server (Xunfei Cloud). POST /speak or GET /speak?text=..."


# ========== 启动 ==========
def main():
    _init_player()

    t = threading.Thread(target=_tts_worker, daemon=True)
    t.start()

    flask_thread = threading.Thread(
        target=lambda: flask_app.run(host="0.0.0.0", port=HTTP_PORT, use_reloader=False),
        daemon=True
    )
    flask_thread.start()
    print(f"[TTS-Server] HTTP :{HTTP_PORT}")

    if MAIX_AVAILABLE:
        cam = camera.Camera(640, 480)
        disp = display.Display()
        while not app.need_exit():
            img = cam.read()
            disp.show(img)
    else:
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
