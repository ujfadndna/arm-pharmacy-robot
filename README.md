# AI智能药房末端执行系统

> **2026年全国大学生电子设计竞赛信息科技前沿专题赛（瑞萨杯）**
> 基于Eye-in-Hand视觉伺服的AI智能药房末端执行系统

---

## 项目简介

本项目是一套以**RA6M5 + MaixCAM2**为核心的6-DOF机械臂智能控制系统，面向药房自动化场景。

核心功能：
- **自然语言取药**：语音指令 → LLM解析 → 机械臂自动取药
- **AI药柜整理**：视觉扫描药柜 → LLM规划 → 自动整理归位
- **Eye-in-Hand视觉伺服**：AprilTag精确定位 + 迭代视觉纠偏

---

## 硬件架构

```
MaixCAM2 (AI边缘节点)
  ├── 语音识别 (讯飞ASR)
  ├── 云端LLM (火山引擎DeepSeek)
  ├── AprilTag视觉定位
  └── UART ──────────────────► RA6M5 (Cortex-M33, 200MHz)
                                  ├── 逆运动学 + 轨迹规划
                                  ├── CAN FD ──► ZDT闭环步进电机 ×6
                                  └── PWM ─────► 夹爪舵机
```

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控MCU | RA6M5 (Cortex-M33, 200MHz) | 百问网 DShanMCU-RA6M5 |
| AI边缘节点 | MaixCAM2 (K230) | 语音+视觉+LLM网关 |
| 电机驱动 | ZDT闭环步进 ZDT_X_V2 | CAN FD通信，内置堵转保护 |
| 机械臂 | 6-DOF桌面机械臂 | 臂展约400mm |
| 夹爪 | 舵机夹爪 (25kg) | PWM控制 |

---

## 软件架构

```
LED_RTOS_keil/src/
├── 运动控制层
│   ├── motor_can.c        # ZDT电机CAN驱动
│   ├── kinematics.c       # 6-DOF逆运动学（解析解）
│   ├── trajectory.c       # 轨迹规划（Minimum Jerk）
│   └── motion_controller.c # 运动状态机
│
├── 任务执行层
│   ├── cabinet_main.c     # 主状态机（扫描→思考→执行）
│   ├── cabinet_executor.c # 抓取序列（接近→抓取→归位）
│   ├── cabinet_config.c   # 药柜几何参数与标定
│   └── visual_servo.c     # AprilTag视觉伺服纠偏
│
├── AI集成层
│   ├── llm_action.c       # LLM响应解析（Function Calling）
│   ├── vision_wifi.c      # MaixCAM2 WiFi视觉通信
│   └── w800_driver.c      # W800 WiFi模块驱动
│
└── 可靠性层
    ├── degradation.c      # 5级降级策略
    ├── realtime_monitor.c # 实时性监控
    └── watchdog.c         # 看门狗

MaixCam2/
├── main_vision.py         # 主程序（视觉+语音+LLM）
├── cabinet_scanner.py     # 药柜AprilTag扫描
├── voice_llm_chat.py      # 语音对话（讯飞ASR + DeepSeek）
├── visual_servo.py        # 视觉伺服反馈
└── config_example.py      # API配置模板
```

---

## 快速开始

### 1. RA6M5 固件（LED_RTOS_keil）

**环境要求**：Keil MDK 5.38+，Renesas FSP 5.x

```bash
# 用Keil打开项目
LED_RTOS_keil/FSP_Project.uvprojx

# 编译并烧录到RA6M5
Project → Build → Flash Download
```

### 2. MaixCAM2 视觉程序

**环境要求**：MaixCAM2 + MaixPy SDK

```bash
cd MaixCam2/

# 配置API key
cp config_example.py config.py
# 编辑 config.py 填入真实的讯飞和火山引擎API key

# 部署到MaixCAM2
# 通过MaixPy IDE或scp上传至设备 /root/
```

### 3. 手眼标定

```
1. 将AprilTag贴在药柜各格子上
2. 运行 hand_eye_calibrate.py 采集标定点
3. 标定结果自动保存，重启后生效
```

---

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 控制频率 | 200Hz | GPT0定时器，5ms周期 |
| CAN波特率 | 500Kbps / 2Mbps FD | 标准段/数据段 |
| 视觉伺服精度 | ±5mm | AprilTag定位，3次迭代 |
| 接近高度 | 60mm | 抓取前悬停高度 |
| 降级策略 | 5级 | 单关节故障→全局急停 |

---

## 目录结构

```
Arm/
├── LED_RTOS_keil/          # RA6M5嵌入式工程（Keil）
│   ├── src/                # 应用源码
│   ├── ra/                 # Renesas FSP驱动
│   ├── ra_cfg/             # 硬件配置
│   └── ra_gen/             # FSP自动生成代码
├── MaixCam2/               # MaixCAM2 Python程序
│   ├── main_vision.py      # 主程序入口
│   ├── config_example.py   # API配置模板
│   └── ...
├── CLAUDE.md               # 项目技术文档
└── README.md               # 本文件
```

---

## 技术栈

| 层次 | 技术 |
|------|------|
| 嵌入式 | C / FreeRTOS / Renesas FSP / CAN FD |
| 算法 | 解析IK / Minimum Jerk轨迹 / AprilTag |
| AI | DeepSeek LLM / 讯飞ASR / Function Calling |
| 视觉 | MaixPy / AprilTag / Eye-in-Hand标定 |

---

## License

MIT License
