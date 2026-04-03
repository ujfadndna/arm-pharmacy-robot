"""
test_w800_udp.py - 结果写入 /tmp/w800_test.log，断线后也能查看
"""

import socket, struct, time, subprocess, os, signal

LOG = "/root/w800_test.log"
AP_SSID    = "RA6M5-Arm"
AP_PWD     = "12345678"
W800_IP    = "192.168.4.1"
W800_PORT  = 8888
LOCAL_PORT = 8889

def log(msg):
    print(msg)
    with open(LOG, "a") as f:
        f.write(msg + "\n")

def run(cmd, timeout=5):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except:
        return ""

def wpa(args):
    return run(f"wpa_cli -i wlan0 {args}")

def get_ip():
    out = run("ip addr show wlan0")
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("inet ") and not line.startswith("inet6"):
            return line.split()[1].split("/")[0]
    return None

# 清空旧日志
open(LOG, "w").close()
log(f"=== W800 UDP 测试 {time.strftime('%H:%M:%S')} ===")
log(f"当前 IP: {get_ip()}")

# 1. 切换 WiFi
log(f"\n[WiFi] 连接 {AP_SSID} ...")
net_list = wpa("list_networks")
net_id = None
for line in net_list.splitlines()[1:]:
    if AP_SSID in line:
        net_id = line.split("\t")[0].strip()
        break

if net_id is None:
    net_id = wpa("add_network").strip()
    wpa(f'set_network {net_id} ssid \\"{AP_SSID}\\"')
    wpa(f'set_network {net_id} psk \\"{AP_PWD}\\"')
    log(f"[WiFi] 新建网络 id={net_id}")
else:
    log(f"[WiFi] 已有网络 id={net_id}")

wpa(f"select_network {net_id}")
log("[WiFi] 等待关联...")

for i in range(20):
    status = wpa("status")
    log(f"  [{i}] {status[:60].replace(chr(10),' ')}")
    if "COMPLETED" in status and AP_SSID in status:
        log("[WiFi] 关联成功！")
        break
    time.sleep(1)

time.sleep(2)

# 2. 强制 DHCP 更新
log("[DHCP] 释放旧 IP 并重新申请...")
run("ip addr flush dev wlan0")
run("udhcpc -i wlan0 -n -q -t 8", timeout=20)

for i in range(15):
    ip = get_ip()
    log(f"  [{i}] 当前 IP: {ip}")
    if ip and ip.startswith("192.168.4."):
        log(f"[DHCP] 获得目标 IP: {ip}")
        break
    time.sleep(1)

ip = get_ip()
log(f"\n最终 IP: {ip}")

# 3. 发 UDP
log(f"\n[UDP] 发送到 {W800_IP}:{W800_PORT}")
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    def frame(cmd, data=b''):
        chk = cmd ^ (len(data)&0xFF) ^ ((len(data)>>8)&0xFF)
        for b in data: chk ^= b
        return bytes([0xAA,0x55,cmd])+struct.pack('<H',len(data))+data+bytes([chk&0xFF])

    for i in range(5):
        f = frame(0x01)
        sock.sendto(f, (W800_IP, W800_PORT))
        log(f"  心跳 {i+1}: {f.hex().upper()}")
        time.sleep(0.5)

    qr = b"DRUG_TEST"
    f = frame(0x03, bytes([len(qr)])+qr)
    sock.sendto(f, (W800_IP, W800_PORT))
    log(f"  QR: {f.hex().upper()}")

    sock.close()
    log("[UDP] 发送完成")
except Exception as e:
    log(f"[UDP] 错误: {e}")

log(f"\n=== 完成 {time.strftime('%H:%M:%S')} ===")
log(f"日志已写入 {LOG}")
log("请重启 MaixCam2 恢复 MaixVision 连接，然后读取此日志")
