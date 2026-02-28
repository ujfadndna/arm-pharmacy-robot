# AI智能药房末端执行系统

## 项目概述

- **比赛**: 2026年全国大学生电子设计竞赛信息科技前沿专题赛（瑞萨杯）
- **全称**: 基于Eye-in-Hand视觉伺服的AI智能药房末端执行系统
- **方向**: AI具身智能 + "眼手力"机器人作业
- **应用场景**: 药房自动取药 + AI智能药柜整理
- **核心亮点**: LLM驱动的自然语言任务规划

---

## 硬件架构

```
MaixCam2 (AI边缘节点)  ──UART(二进制帧)──►  RA6M5 (实时执行层)  ──CAN──►  ZDT_X_V2 (6轴电机)
     │                                        │
     │ 语音识别 + 云端LLM                      │ PWM
     │ AprilTag + QR扫描                       ▼
     │ WiFi远程监控                         夹爪舵机 + 串口屏
     └────────────────────────────────────────────────────────
```

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控MCU | RA6M5 (Cortex-M33, 200MHz) | 百问网 DShanMCU-RA6M5 |
| AI边缘节点 | MaixCam2 (K230) | 语音+视觉+LLM网关 |
| 电机驱动 | ZDT闭环步进 + **ZDT_X_V2固件** | CAN通信，支持堵转保护 |
| 机械臂 | ZERO机械臂复刻版 | 6-DOF，DH参数见ZERO项目 |
| 夹爪 | Episode1套件 (25kg舵机) | PWM控制 |
| 显示 | 淘晶驰串口屏 4.3寸 | 仪表盘状态显示 |

---

## 关键技术决策

| 决策点 | 选择 | 理由 |
|--------|------|------|
| RTOS | FreeRTOS | 百问网教程是FreeRTOS，无需移植 |
| 电机固件 | ZDT_X_V2 | 支持堵转保护，比Emm_V5更适合机械臂 |
| **AI交互** | **云端LLM + Function Calling** | **核心创新点，具身智能闭环** |
| **语音识别** | **Whisper (本地/云端)** | **MaixCam2内置麦克风，0-30cm有效** |
| 碰撞检测 | ZDT堵转保护 | 硬件检测<10ms，比软件读电流可靠 |
| 轨迹规划 | Minimum Jerk | 比S曲线简单，数学最优平滑 |
| 通信协议 | 二进制帧 0xAA 0x55 | 比文本协议效率高，防粘包 |
| Web监控 | MaixCam2 WiFi | 展示层补强，不改RA6M5硬件 |

---

## AI功能模块

### AI智能药柜整理（核心亮点）

```
用户语音 → MaixCam2 ASR → 云端LLM → JSON任务计划 → RA6M5执行 → 语音反馈
```

**技术栈**:
- 语音识别: Whisper (MaixCam2本地或云端)
- LLM: DeepSeek/OpenAI API (Function Calling)
- 药品识别: QR Code扫描 (药品ID、有效期、位置)
- 执行: 复用取药流程

**演示场景**:
> 用户: "小药，帮我整理药柜，把快过期的药放前面"
> 系统: 扫描 → LLM规划 → 执行整理 → "整理完成，2盒药即将过期"

---

## 规划文档

**详细规划见**: `ACTION_PLAN.md` (v2.0)

### 开发优先级

| 优先级 | 模块 | 工作量 |
|--------|------|--------|
| P0 | CAN通信 + 电机控制 | 1周 |
| P0 | 逆运动学 + 轨迹规划 | 1周 |
| P0 | AprilTag定位 + 手眼标定 | 1周 |
| P1 | **AI语音交互 + LLM集成** | **1周** |
| P1 | **AI智能药柜整理** | **1周** |
| P1 | 碰撞检测 + 柔顺放置 | 3天 |
| P2 | Web监控 + 串口屏 | 3天 |

---

## 开发环境

| 环境 | 路径 |
|------|------|
| **keil工作区** | `C:\Users\21181\e2_studio\workspace\LED_RTOS` |

---

## 实时性与可靠性模块

### 延迟预算（必读）

**文档**: `LED_RTOS_keil/doc/LATENCY_BUDGET.md`

| 环节 | 预算 | 说明 |
|------|------|------|
| 运动控制周期 | 5ms | GPT0定时器，硬实时 |
| Deadline | 6ms | 超过触发miss计数 |
| CAN超时 | 500ms | 电机失联检测阈值 |
| 碰撞检测 | 20ms | 4个tick检测一次 |
| 看门狗 | 2.7s | 系统卡死自动复位 |

### 日志模块

**文件**: `src/log.h`, `src/log.c`

```c
#include "log.h"
LOG_INFO("MOTOR", "Joint %d reached", joint_id);
LOG_WARN("CAN", "Timeout on J%d", id);
LOG_ERROR("MOTION", "IK failed");
```

级别: `LOG_DEBUG` < `LOG_INFO` < `LOG_WARN` < `LOG_ERROR`

### 降级策略

**文件**: `src/degradation.h`, `src/degradation.c`

```
DEGRADE_NONE → DEGRADE_SLOW → DEGRADE_SINGLE_JOINT → DEGRADE_POSITION_HOLD → DEGRADE_EMERGENCY
```

- 单关节CAN超时 → 禁用该关节继续运行
- 多关节超时/连续失败 → 急停
- 堵转 → 位置保持等待clear

### 调试命令

| 命令 | 作用 |
|------|------|
| `rtmon` | 查看周期/抖动/deadline miss |
| `taskmon` | 查看CPU占用 |
| `degrade` | 查看降级状态 |
| `clear` | 清除故障恢复 |

---

## 驱动开发参考

- **映射表**: @.claude/embedded/driver_reference.md
- **硬件规格**: @.claude/embedded/hardware_spec.md

---

## 参考资源

| 资源 | 路径 |
|------|------|
| **行动规划** | `ACTION_PLAN.md` |
| ZERO逆运动学 | `Doc/Reference/zero-robotic-arm-master/2. Software/robot/Core/Src/robot_kinematics.c` |
| ZDT_X_V2协议 | `Doc/Reference/ZDT闭环步进资料/ZDT_X_V2.0闭环步进资料/` |
| 百问网教程 | `Doc/Reference/DShanMCU-RA6M5配套学习资料/` |
| MaixPy SDK | `Doc/Reference/MaixPy-main/` |
| RA6M5数据手册 | `Doc/Datasheets/r01ds0366ej0140-ra6m5.pdf` |
| RA6M5用户手册 | `Doc/Datasheets/r01uh0891ej0140-ra6m5.pdf` |

---

*最后更新: 2026-01-30*
*规划版本: ACTION_PLAN.md v2.0*
