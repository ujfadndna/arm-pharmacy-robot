"""
voice_llm_chat.py - MaixCam2 语音+云端LLM对话 (触摸屏版)

功能：
1. 触摸屏按钮（按住说话）
2. 讯飞ASR一次性识别（录完再发）
3. 火山引擎DeepSeek云端LLM对话
4. MeloTTS语音播报
5. UART发送动作指令给RA6M5

流程：
按住按钮 → 录音 → 松开 → ASR识别 → LLM理解 → TTS播报
"""

import time
import hashlib
import hmac
import base64
import json
import socket
import ssl
import struct
import os
import http.client
from enum import Enum
from urllib.parse import urlencode, urlparse

# ==================== 平台检测 ====================
try:
    from maix import nn, audio, app, uart, pinmap, sys, err, display, image, touchscreen
    MAIX_AVAILABLE = True
except ImportError:
    MAIX_AVAILABLE = False
    print("[WARN] maix not available, using mock mode")


# ==================== 配置 ====================
# API key等敏感配置从config.py读取（config.py已加入.gitignore）
# 首次使用：cp config_example.py config.py 并填入真实key
try:
    from config import XFYUN_APP_ID, XFYUN_API_KEY, XFYUN_API_SECRET
    from config import ARK_API_KEY, ARK_BASE_URL, ARK_MODEL
except ImportError:
    raise RuntimeError("缺少config.py，请参考config_example.py创建")

# TTS 配置
MELOTTS_MODEL = "/root/models/melotts/melotts-zh.mud"
TTS_SAMPLE_RATE = 44100
TTS_VOLUME = 80
TTS_SPEED = 0.8

# 录音配置
RECORD_SAMPLE_RATE = 16000
RECORD_CHANNELS = 1
MAX_RECORD_SECONDS = 10  # 最长录音时间

# System Prompt (药柜助手 + 通用对话)
SYSTEM_PROMPT = """你是智能药房助手小药，既能管理药柜，也能和用户闲聊。

## 药柜操作
当用户需要取药、放药、整理药柜时，输出JSON动作序列：

可用动作:
- fetch(slot): 取出药品，如 {"action":"fetch","slot":"A1"}
- move(from,to): 移动药品，如 {"action":"move","from":"A1","to":"B2"}
- return(slot): 放回药品
- speak(text): 语音播报

药柜布局 (4x4):
A1 A2 A3 A4
B1 B2 B3 B4
C1 C2 C3 C4
D1 D2 D3 D4

药柜操作示例: [{"action":"fetch","slot":"A1"},{"action":"speak","text":"已取出阿莫西林"}]

## 闲聊模式
当用户只是聊天、问问题时，直接用speak回复，像朋友一样自然对话。
示例: [{"action":"speak","text":"勒布朗詹姆斯是NBA传奇球星，效力于湖人队"}]

注意：始终输出JSON数组格式，不要解释。
"""


# ==================== 简易 WebSocket ====================
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

    def send_binary(self, data):
        if not self.connected:
            return False
        try:
            frame = self._create_frame(data, opcode=0x02)
            self.sock.sendall(frame)
            return True
        except:
            self.connected = False
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

    def recv(self, timeout=5):
        if not self.connected:
            return None
        try:
            self.sock.settimeout(timeout)
            data = self.sock.recv(8192)
            if not data:
                return None
            return self._parse_frame(data)
        except socket.timeout:
            return None
        except Exception as e:
            print(f"[WS] Recv error: {e}")
            return None

    def _parse_frame(self, data):
        if len(data) < 2:
            return None
        payload_len = data[1] & 0x7F
        offset = 2
        if payload_len == 126:
            if len(data) < 4:
                return None
            payload_len = struct.unpack(">H", data[2:4])[0]
            offset = 4
        elif payload_len == 127:
            if len(data) < 10:
                return None
            payload_len = struct.unpack(">Q", data[2:10])[0]
            offset = 10

        if len(data) < offset + payload_len:
            return None
        payload = data[offset:offset + payload_len]
        try:
            return payload.decode('utf-8')
        except:
            return None

    def close(self):
        self.connected = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None


# ==================== 讯飞 ASR (一次性识别) ====================
class XfyunASR:
    """讯飞语音识别 - 录完再发模式"""
    HOST = "iat-api.xfyun.cn"
    PATH = "/v2/iat"

    def __init__(self):
        pass

    def _create_url(self):
        from datetime import datetime, timezone
        now = datetime.now(timezone.utc)
        date = now.strftime('%a, %d %b %Y %H:%M:%S GMT')

        signature_origin = f"host: {self.HOST}\ndate: {date}\nGET {self.PATH} HTTP/1.1"
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
            "host": self.HOST
        }
        return f"wss://{self.HOST}{self.PATH}?{urlencode(params)}"

    def recognize(self, audio_data):
        """
        一次性识别音频数据
        audio_data: PCM 16bit 16kHz 单声道
        返回: 识别文本 或 None
        """
        if not audio_data or len(audio_data) < 1000:
            print("[ASR] Audio too short")
            return None

        print(f"[ASR] Recognizing {len(audio_data)} bytes...")

        try:
            url = self._create_url()
            ws = SimpleWebSocket()
            if not ws.connect(url, timeout=10):
                print("[ASR] WebSocket connect failed")
                return None

            # 分帧发送（每帧1280字节 = 40ms）
            frame_size = 1280
            total_frames = (len(audio_data) + frame_size - 1) // frame_size
            result_text = ""

            for i in range(total_frames):
                start = i * frame_size
                end = min(start + frame_size, len(audio_data))
                chunk = audio_data[start:end]
                audio_base64 = base64.b64encode(chunk).decode('utf-8')

                if i == 0:
                    # 第一帧
                    frame = {
                        "common": {"app_id": XFYUN_APP_ID},
                        "business": {
                            "language": "zh_cn",
                            "domain": "iat",
                            "accent": "mandarin",
                            "vad_eos": 3000
                        },
                        "data": {
                            "status": 0,
                            "format": "audio/L16;rate=16000",
                            "encoding": "raw",
                            "audio": audio_base64
                        }
                    }
                elif i == total_frames - 1:
                    # 最后一帧
                    frame = {
                        "data": {
                            "status": 2,
                            "format": "audio/L16;rate=16000",
                            "encoding": "raw",
                            "audio": audio_base64
                        }
                    }
                else:
                    # 中间帧
                    frame = {
                        "data": {
                            "status": 1,
                            "format": "audio/L16;rate=16000",
                            "encoding": "raw",
                            "audio": audio_base64
                        }
                    }

                ws.send_text(json.dumps(frame))
                time.sleep(0.02)  # 稍微等一下，避免发太快

            # 等待结果
            print("[ASR] Waiting for result...")
            for _ in range(50):  # 最多等5秒
                msg = ws.recv(timeout=0.1)
                if msg:
                    try:
                        data = json.loads(msg)
                        code = data.get("code", 0)
                        if code != 0:
                            print(f"[ASR] Error code: {code}")
                            break

                        result = data.get("data", {}).get("result", {})
                        ws_list = result.get("ws", [])
                        for ws_item in ws_list:
                            for cw in ws_item.get("cw", []):
                                result_text += cw.get("w", "")

                        # 检查是否结束
                        status = data.get("data", {}).get("status", 0)
                        if status == 2:
                            break
                    except:
                        pass

            ws.close()

            if result_text:
                print(f"[ASR] Result: {result_text}")
            else:
                print("[ASR] No result")
            return result_text if result_text else None

        except Exception as e:
            print(f"[ASR] Error: {e}")
            return None


# ==================== 火山引擎 LLM ====================
class ArkLLM:
    def __init__(self):
        self.conversation = []

    def chat(self, user_message, cabinet_state=None):
        if cabinet_state:
            full_message = f"药柜状态:\n{cabinet_state}\n\n用户指令: {user_message}"
        else:
            full_message = user_message

        self.conversation.append({"role": "user", "content": full_message})
        messages = [{"role": "system", "content": SYSTEM_PROMPT}] + self.conversation[-6:]

        try:
            conn = http.client.HTTPSConnection(ARK_BASE_URL, timeout=30)
            body = json.dumps({
                "model": ARK_MODEL,
                "messages": messages,
                "max_tokens": 300,
                "temperature": 0.3
            })
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {ARK_API_KEY}"
            }

            conn.request("POST", "/api/v3/chat/completions", body, headers)
            response = conn.getresponse()
            data = response.read().decode('utf-8')
            conn.close()

            result = json.loads(data)
            if "choices" in result and len(result["choices"]) > 0:
                content = result["choices"][0]["message"]["content"]
                self.conversation.append({"role": "assistant", "content": content})
                return content
            else:
                print(f"[LLM] Unexpected response: {data[:200]}")
                return None

        except Exception as e:
            print(f"[LLM] Error: {e}")
            return None

    def clear_history(self):
        self.conversation = []


# ==================== 状态枚举 ====================
class VoiceState(Enum):
    IDLE = 0        # 等待按钮
    RECORDING = 1   # 录音中
    THINKING = 2    # 处理中
    SPEAKING = 3    # 播报中


# ==================== 主控制器 ====================
class VoiceLLMChat:
    def __init__(self):
        self.state = VoiceState.IDLE
        self.asr = None
        self.llm = None
        self.tts = None
        self.player = None
        self.serial = None
        self.disp = None
        self.ts = None
        self._running = False

        # 录音相关
        self._recorder = None
        self._audio_buffer = bytearray()

        # 按钮位置
        self.btn_pos = [0, 0, 0, 0]  # x, y, w, h

    def init(self):
        print("[VoiceLLM] Initializing...")

        if MAIX_AVAILABLE:
            # 显示屏
            self.disp = display.Display()
            # 触摸屏
            self.ts = touchscreen.TouchScreen()
            # TTS
            self._init_tts()
            # UART
            self._init_uart()

        # ASR & LLM
        self.asr = XfyunASR()
        self.llm = ArkLLM()

        # 计算按钮位置（屏幕中下方）
        if self.disp:
            btn_w = 200
            btn_h = 80
            self.btn_pos = [
                (self.disp.width() - btn_w) // 2,
                self.disp.height() - btn_h - 50,
                btn_w,
                btn_h
            ]

        print("[VoiceLLM] Init complete")
        return True

    def _init_tts(self):
        if not MAIX_AVAILABLE:
            return True
        try:
            self.tts = nn.MeloTTS(model=MELOTTS_MODEL, speed=TTS_SPEED, language='zh')
            self.player = audio.Player(sample_rate=TTS_SAMPLE_RATE, block=False)
            self.player.volume(TTS_VOLUME)
            return True
        except Exception as e:
            print(f"[TTS] Init failed: {e}")
            return False

    def _init_uart(self):
        if not MAIX_AVAILABLE:
            return
        try:
            device_id = sys.device_id()
            if device_id == "maixcam2":
                pin_function = {"A21": "UART4_TX", "A22": "UART4_RX"}
                device = "/dev/ttyS4"
            else:
                pin_function = {"A16": "UART0_TX", "A17": "UART0_RX"}
                device = "/dev/ttyS0"

            for pin, func in pin_function.items():
                err.check_raise(pinmap.set_pin_function(pin, func),
                                f"Failed set pin {pin} to {func}")

            self.serial = uart.UART(device, 115200)
            print(f"[UART] {device} @ 115200")
        except Exception as e:
            print(f"[UART] Init failed: {e}")

    def speak(self, text, block=True):
        if not text:
            return
        print(f"[TTS] Speaking: {text}")
        if not MAIX_AVAILABLE or not self.tts:
            time.sleep(len(text) * 0.15)
            print("[TTS] Done (mock)")
            return
        try:
            pcm_data = self.tts.infer(text, output_pcm=True)
            if pcm_data:
                self.player.play(pcm_data)
                if block:
                    # 等待播放完成
                    wait_time = len(text) * 0.15 + 0.5
                    print(f"[TTS] Waiting {wait_time:.1f}s...")
                    time.sleep(wait_time)
            print("[TTS] Done")
        except Exception as e:
            print(f"[TTS] Error: {e}")

    def send_uart(self, command):
        print(f"[UART] TX: {command}")
        if self.serial:
            try:
                self.serial.write_str(command + "\n")
            except:
                pass

    def is_in_button(self, x, y):
        return (self.btn_pos[0] <= x <= self.btn_pos[0] + self.btn_pos[2] and
                self.btn_pos[1] <= y <= self.btn_pos[1] + self.btn_pos[3])

    def draw_ui(self, img):
        """绘制界面"""
        # 清空
        img.clear()

        # 标题
        img.draw_string(10, 10, "智能药房助手", image.COLOR_WHITE, scale=2)

        # 状态文字
        status_text = {
            VoiceState.IDLE: "按住按钮说话",
            VoiceState.RECORDING: "正在录音...",
            VoiceState.THINKING: "思考中...",
            VoiceState.SPEAKING: "播报中..."
        }
        img.draw_string(10, 60, status_text.get(self.state, ""), image.COLOR_GREEN, scale=1.5)

        # 按钮
        btn_color = image.COLOR_RED if self.state == VoiceState.RECORDING else image.COLOR_BLUE
        img.draw_rect(self.btn_pos[0], self.btn_pos[1],
                      self.btn_pos[2], self.btn_pos[3], btn_color, -1)
        img.draw_rect(self.btn_pos[0], self.btn_pos[1],
                      self.btn_pos[2], self.btn_pos[3], image.COLOR_WHITE, 2)

        # 按钮文字
        btn_text = "松开发送" if self.state == VoiceState.RECORDING else "按住说话"
        text_x = self.btn_pos[0] + 40
        text_y = self.btn_pos[1] + 25
        img.draw_string(text_x, text_y, btn_text, image.COLOR_WHITE, scale=1.5)

    def start_recording(self):
        """开始录音"""
        if not MAIX_AVAILABLE:
            return
        try:
            self._audio_buffer = bytearray()
            self._recorder = audio.Recorder(sample_rate=RECORD_SAMPLE_RATE, channel=RECORD_CHANNELS)
            self._recorder.volume(100)
            print("[REC] Started")
        except Exception as e:
            print(f"[REC] Start failed: {e}")

    def stop_recording(self):
        """停止录音，返回音频数据"""
        if not MAIX_AVAILABLE or not self._recorder:
            return bytes(self._audio_buffer)
        try:
            self._recorder.finish()
            self._recorder = None
            print(f"[REC] Stopped, {len(self._audio_buffer)} bytes")
        except:
            pass
        return bytes(self._audio_buffer)

    def record_chunk(self):
        """录制一小段音频"""
        if not MAIX_AVAILABLE or not self._recorder:
            return
        try:
            data = self._recorder.record(40)  # 40ms
            if data:
                self._audio_buffer.extend(data)
                # 检查是否超过最大时长
                max_bytes = MAX_RECORD_SECONDS * RECORD_SAMPLE_RATE * 2
                if len(self._audio_buffer) >= max_bytes:
                    print("[REC] Max duration reached")
                    return False
        except:
            pass
        return True

    def parse_llm_response(self, response):
        """解析LLM响应"""
        actions = []
        speak_text = ""

        try:
            response = response.strip()
            if response.startswith("```"):
                lines = response.split("\n")
                response = "\n".join(lines[1:-1] if lines[-1] == "```" else lines[1:])

            data = json.loads(response)

            if isinstance(data, list):
                for item in data:
                    action = item.get("action", "")
                    if action == "speak":
                        speak_text = item.get("text", "")
                    else:
                        actions.append(item)
            elif isinstance(data, dict):
                action = data.get("action", "")
                if action == "speak":
                    speak_text = data.get("text", "")
                else:
                    actions.append(data)

        except json.JSONDecodeError:
            speak_text = response

        return actions, speak_text

    def process_audio(self, audio_data):
        """处理录音：ASR → LLM → TTS"""
        self.state = VoiceState.THINKING

        # ASR识别
        text = self.asr.recognize(audio_data)
        if not text:
            self.speak("没听清，请再说一次")
            self.state = VoiceState.IDLE
            return

        print(f"[User] {text}")

        # LLM处理
        self.speak("让我想想")
        response = self.llm.chat(text)

        if response:
            print(f"[LLM] {response}")
            actions, speak_text = self.parse_llm_response(response)

            # 发送动作
            for action in actions:
                action_type = action.get("action", "")
                if action_type == "fetch":
                    slot = action.get("slot", "")
                    self.send_uart(f"V:FETCH,{slot}")
                elif action_type == "move":
                    from_slot = action.get("from", "")
                    to_slot = action.get("to", "")
                    self.send_uart(f"V:MOVE,{from_slot},{to_slot}")
                elif action_type == "return":
                    slot = action.get("slot", "")
                    self.send_uart(f"V:RETURN,{slot}")

            # 播报
            self.state = VoiceState.SPEAKING
            if speak_text:
                self.speak(speak_text)
            elif not actions:
                self.speak("好的")
        else:
            self.speak("抱歉，我没理解")

        print("[VoiceLLM] Back to IDLE")
        self.state = VoiceState.IDLE

    def run(self):
        print("\n" + "=" * 50)
        print("  MaixCam2 Voice + LLM Chat (Touch)")
        print("  Press and hold button to speak")
        print("=" * 50 + "\n")

        self._running = True
        self.state = VoiceState.IDLE

        if MAIX_AVAILABLE:
            img = image.Image(self.disp.width(), self.disp.height())
        else:
            img = None

        self.speak("欢迎使用智能药房")

        last_pressed = False

        while self._running:
            try:
                if MAIX_AVAILABLE and app.need_exit():
                    break

                # 读取触摸
                if self.ts:
                    x, y, pressed = self.ts.read()
                else:
                    pressed = False
                    x, y = 0, 0

                # 状态机
                if self.state == VoiceState.IDLE:
                    # 检测按下
                    if pressed and not last_pressed and self.is_in_button(x, y):
                        print("[UI] Button pressed, start recording")
                        self.state = VoiceState.RECORDING
                        self.start_recording()

                elif self.state == VoiceState.RECORDING:
                    # 持续录音
                    if not self.record_chunk():
                        # 超时，自动停止
                        print("[UI] Recording timeout")
                        audio_data = self.stop_recording()
                        self.process_audio(audio_data)
                    elif not pressed:
                        # 松开按钮
                        print("[UI] Button released")
                        audio_data = self.stop_recording()
                        if len(audio_data) > 3200:  # 至少100ms
                            self.process_audio(audio_data)
                        else:
                            print("[REC] Too short, ignored")
                            self.state = VoiceState.IDLE

                last_pressed = pressed

                # 绘制UI
                if img and self.disp:
                    self.draw_ui(img)
                    self.disp.show(img)

                time.sleep(0.02)

            except KeyboardInterrupt:
                print("\n[Interrupted]")
                break
            except Exception as e:
                print(f"[Error] {e}")
                time.sleep(1)

        self._running = False
        print("[VoiceLLM] Stopped")


# ==================== 主程序 ====================
def main():
    print("=" * 50)
    print("  MaixCam2 Voice + Cloud LLM Chat")
    print("  ASR: Xfyun | LLM: DeepSeek | TTS: MeloTTS")
    print("=" * 50)

    chat = VoiceLLMChat()
    if not chat.init():
        print("Init failed!")
        return
    chat.run()


if __name__ == "__main__":
    main()
