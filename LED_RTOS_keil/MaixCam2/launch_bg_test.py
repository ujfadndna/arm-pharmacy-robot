"""
launch_bg_test.py - 把 bg_w800_test.py 上传到设备并后台启动
MaixVision 保持连接，后台进程独立运行
"""
import subprocess, time, os

# 启动后台进程（nohup 防止进程被 MaixVision 关闭时一起杀掉）
proc = subprocess.Popen(
    ['nohup', 'python3', '/root/bg_w800_test.py'],
    stdout=open('/root/bg_stdout.log', 'w'),
    stderr=subprocess.STDOUT,
    close_fds=True
)
print(f"[OK] 后台进程已启动，PID={proc.pid}")
print("等待 40 秒后读取日志...")
time.sleep(40)

print("\n=== /root/w800_test.log ===")
try:
    print(open('/root/w800_test.log').read())
except:
    print("(日志文件不存在)")

print("\n=== stdout ===")
try:
    print(open('/root/bg_stdout.log').read())
except:
    print("(无 stdout)")
