#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
speak_client.py - 发送播报文本到 MaixCam2

用法:
  python tools/speak_client.py --ip 192.168.x.x --text "正在抓取阿莫西林"

导入:
  from tools.speak_client import send_speak
  send_speak("192.168.x.x", "正在抓取阿莫西林")
"""

import argparse
import urllib.request


def send_speak(maixcam_ip: str, text: str, port: int = 8000) -> bool:
    """发送播报文本到 MaixCam2 TTS 服务。"""
    url = f"http://{maixcam_ip}:{port}/speak"
    data = text.encode("utf-8")
    try:
        req = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": "text/plain; charset=utf-8"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            print(f"[Speak] -> {text}  ({resp.read().decode()})")
            return True
    except Exception as ex:
        print(f"[Speak] Failed: {ex}")
        return False


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Send text to MaixCam2 TTS")
    p.add_argument("--ip", required=True)
    p.add_argument("--text", required=True)
    p.add_argument("--port", type=int, default=8000)
    args = p.parse_args()
    if not send_speak(args.ip, args.text, args.port):
        raise SystemExit(1)
