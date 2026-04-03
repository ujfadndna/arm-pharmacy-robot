# config.py - 本地配置文件（已加入.gitignore，不会提交到GitHub）
# 请勿分享此文件

# 讯飞 ASR 配置
XFYUN_APP_ID     = "98d6a094"
XFYUN_API_KEY    = "02f3d11253c7d03fc036b7ddc2f5da07"
XFYUN_API_SECRET = "MWQ1ZjhjYjMwOGVmZWYyNzk1MGUxZDNm"

# 火山引擎 DeepSeek 配置
ARK_API_KEY  = "174d3637-de57-415a-8528-917586c9b248"
ARK_BASE_URL = "ark.cn-beijing.volces.com"
ARK_MODEL    = "deepseek-v3-250324"

# W800 UDP Bridge 配置
# W800 连接手机热点后的 IP，查看 RA6M5 调试串口输出的 "[W800] W800 IP: x.x.x.x"
W800_IP        = "192.168.4.1"      # W800 SoftAP 固定 IP
W800_UDP_PORT  = 8888
LOCAL_UDP_PORT = 8889
