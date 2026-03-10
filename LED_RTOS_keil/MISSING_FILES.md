# Keil项目缺失文件清单

## 需要添加到Keil项目的源文件

### 核心驱动（必须）
- [ ] src/motor_ctrl_step.c  - ZDT_X_V2电机驱动（CtrlStep协议）
- [ ] src/degradation.c      - 降级策略模块
- [ ] src/log.c              - 日志模块

### 监控模块（必须）
- [ ] src/rtmon.c            - 实时性监控（运动控制周期）
- [ ] src/taskmon.c          - 任务监控（CPU占用）

### 应用模块（可选）
- [ ] src/medicine_db.c      - 药品数据库
- [ ] src/cabinet_demo.c     - 药柜演示
- [ ] src/cabinet_executor.c - 药柜执行器
- [ ] src/cabinet_main.c     - 药柜主程序
- [ ] src/motor_sca.c        - SCA驱动（如果不用可删除）
- [ ] src/pid_tuner.c        - PID调参工具
- [ ] src/realtime_monitor.c - 实时监控
- [ ] src/task_monitor.c     - 任务监控
- [ ] src/visual_servo.c     - 视觉伺服

## 如何在Keil中添加文件

### 方法1：通过GUI添加
1. 在Keil中打开项目：FSP_Project.uvprojx
2. 在Project窗口中，右键点击"Source Group 1"
3. 选择"Add Existing Files to Group..."
4. 浏览到src目录，选择上述文件
5. 点击"Add"，然后"Close"

### 方法2：批量添加（推荐）
1. 关闭Keil
2. 备份FSP_Project.uvprojx
3. 运行脚本：tools/add_missing_files_to_keil.py
4. 重新打开Keil项目

## 需要删除的旧文件（可选）

如果确认不使用Emm_V5和SCA协议，可以删除：
- [ ] src/motor_can.c  - Emm_V5协议（旧驱动）
- [ ] src/motor_sca.c  - SCA协议（旧驱动）

**注意**：删除前确保motor_can.c中的CAN中断处理代码已经移植到motor_ctrl_step.c

## 验证步骤

添加文件后，重新编译：
1. 清理项目：Project -> Clean Targets
2. 重新编译：Project -> Build Target (F7)
3. 检查是否还有链接错误

## 当前链接错误清单

### 已解决（文件已创建）
- [x] rtmon_* 系列函数 - rtmon.c已创建
- [x] taskmon_* 系列函数 - taskmon.c已创建

### 待解决（需要添加到项目）
- [ ] motor_ctrl_step_* 系列函数 - 需要添加motor_ctrl_step.c
- [ ] degradation_* 系列函数 - 需要添加degradation.c
- [ ] medicine_db_lookup - 需要添加medicine_db.c
- [ ] debug_printf - 需要检查debug_uart.c
- [ ] joint_config - 在motor_ctrl_step.c中定义

## 编译顺序建议

1. 先添加核心驱动（motor_ctrl_step.c, degradation.c, log.c）
2. 再添加监控模块（rtmon.c, taskmon.c）
3. 最后添加应用模块（medicine_db.c等）
4. 每次添加后编译，逐步解决链接错误
