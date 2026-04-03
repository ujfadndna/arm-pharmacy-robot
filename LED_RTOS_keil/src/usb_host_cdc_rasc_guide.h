/**
 ******************************************************************************
 * @file    usb_host_cdc_rasc_guide.h
 * @brief   RASC配置指引 — USB Host CDC模块添加步骤
 * @note    这不是代码文件，是给开发者的配置说明
 *          完成以下步骤后再编译工程
 ******************************************************************************
 *
 * ======================== 第1步: 打开RASC ========================
 *
 * 方法A: 双击 configuration.xml (如果关联了e2 studio)
 * 方法B: 运行 rasc_launcher.bat
 * 方法C: e2 studio → File → Open → configuration.xml
 *
 * ======================== 第2步: 时钟配置 ========================
 *
 * 进入 Clocks 页面:
 *
 *   当前状态: UCLK = Disabled
 *   目标状态: UCLK = 48MHz
 *
 *   操作:
 *   1. 找到 UCLK (USB Clock) 配置项
 *   2. Source 选择: PLL2
 *      (PLL2 输出 = 24MHz / 2 * 20 = 240MHz, 已配置好)
 *   3. Divider 选择: /5
 *      (240MHz / 5 = 48MHz, 正好是USB要求的时钟)
 *   4. 确认UCLK显示为 48.000 MHz
 *
 *   注意: PLL2同时给CANFD供时钟(240/6=40MHz)，互不影响
 *
 * ======================== 第3步: 添加USB模块栈 ========================
 *
 * 进入 Stacks 页面:
 *
 *   1. 点击 "New Stack" → Connectivity → USB Basic Driver on r_usb_basic
 *      配置属性:
 *        - USB Mode:           Host mode
 *        - USB Speed:          Full Speed
 *        - USB Module Number:  USB_IP0
 *        - Callback:           usb_host_callback
 *
 *   2. 在刚添加的r_usb_basic上右键 → Add → USB HCDC on r_usb_hcdc
 *      (HCDC = Host CDC, 自动挂载在r_usb_basic之上)
 *
 *   3. 确认模块栈结构:
 *      └── Thread (new_thread0 或新建usb_thread)
 *          └── USB Basic Driver (r_usb_basic)
 *              └── USB HCDC (r_usb_hcdc)
 *
 * ======================== 第4步: USB引脚配置 ========================
 *
 * 进入 Pins 页面:
 *
 *   1. 找到 USB0 (USB Full-Speed) 外设
 *   2. 启用以下引脚:
 *      - USB_DP    → 自动分配 (专用引脚)
 *      - USB_DM    → 自动分配 (专用引脚)
 *      - USB_VBUS  → 启用 (VBUS检测输入)
 *      - USB_VBUSEN → 启用 (5V VBUS输出使能, 选一个可用GPIO)
 *
 *   注意: DShanMCU-RA6M5板上USB Host口的引脚已经在PCB上连好，
 *         RASC里只需要启用USB0外设即可
 *
 * ======================== 第5步: 生成代码 ========================
 *
 *   1. 点击 "Generate Project Content" 按钮
 *   2. RASC会在以下位置生成新文件:
 *      - ra_gen/hal_data.h  → 新增 USB 实例声明
 *      - ra_gen/hal_data.c  → 新增 USB 实例定义
 *      - ra_gen/vector_data.h → 新增 USB 中断向量
 *      - ra_cfg/fsp_cfg/r_usb_basic_cfg.h → USB基础配置
 *      - ra/fsp/src/r_usb_basic/ → USB基础驱动源码
 *      - ra/fsp/src/r_usb_hcdc/  → USB Host CDC驱动源码
 *
 * ======================== 第6步: 更新Keil工程 ========================
 *
 *   1. 打开 FSP_Project.uvprojx
 *   2. 添加源文件到工程:
 *      - ra/fsp/src/r_usb_basic/src/ 下所有 .c 文件
 *      - ra/fsp/src/r_usb_hcdc/src/ 下所有 .c 文件
 *      - src/usb_host_cdc.c
 *      - src/dummy_arm_cmd.c
 *   3. 添加头文件路径 (如果需要):
 *      - ra/fsp/inc/api/
 *      - ra/fsp/inc/instances/
 *   4. 确认 Linker scatter file 包含 USB RAM section
 *      (通常RASC会自动更新 fsp_gen.scat)
 *
 * ======================== 第7步: 修改USB实例名 ========================
 *
 *   RASC生成的USB实例名可能是 g_basic0 或其他名称。
 *   打开生成的 ra_gen/hal_data.h，找到USB相关声明，例如:
 *
 *     extern usb_instance_ctrl_t g_basic0_ctrl;
 *     extern const usb_cfg_t     g_basic0_cfg;
 *
 *   如果名称不是 g_basic0，需要修改 usb_host_cdc.c 中的:
 *     extern usb_instance_ctrl_t g_basic0_ctrl;  // 改为实际名称
 *     extern const usb_cfg_t     g_basic0_cfg;   // 改为实际名称
 *
 * ======================== 第8步: TPL配置 ========================
 *
 *   打开 ra_cfg/fsp_cfg/r_usb_basic_cfg.h，确认或添加:
 *
 *     #define USB_CFG_TPLCNT  1
 *     #define USB_CFG_TPL     USB_CFG_TPLCNT, 0x1209, 0x0D32
 *
 *   这告诉USB Host只接受VID=0x1209, PID=0x0D32的设备
 *
 * ======================== 第9步: Composite设备处理 ========================
 *
 *   如果USB枚举失败 (设备插入后没有收到USB_STATUS_CONFIGURED):
 *
 *   原因: STM32是Composite CDC设备 (bDeviceClass=0xEF)，
 *         FSP的HCDC可能只认 bDeviceClass=0x02
 *
 *   修复方案:
 *   找到 ra/fsp/src/r_usb_basic/src/driver/ 下的枚举相关文件
 *   (通常是 r_usb_hmanager.c 或 r_usb_hdriver.c)
 *
 *   搜索 bDeviceClass 的检查代码，将:
 *     if (device_class == USB_IFCLS_CDC)  // 0x02
 *   改为:
 *     if (device_class == USB_IFCLS_CDC || device_class == 0xEF)
 *
 *   或者更精确地，改为VID/PID匹配:
 *     if (vid == 0x1209 && pid == 0x0D32)
 *
 *   备选方案: 如果HCDC驱动改不动，可以绕过它，
 *   直接用 R_USB_PipeRead/R_USB_PipeWrite 操作Bulk管道。
 *   需要手动配置Pipe映射到EP1 OUT(0x01)和EP1 IN(0x81)。
 *
 * ======================== 第10步: 编译测试 ========================
 *
 *   1. 编译工程，修复可能的头文件路径问题
 *   2. 在主任务中添加测试代码:
 *
 *      #include "dummy_arm_cmd.h"
 *
 *      // 在任务函数中:
 *      dummy_arm_init();
 *
 *      if (dummy_arm_wait_ready(10000)) {
 *          LOG_I("MAIN", "Dummy arm connected");
 *          dummy_arm_set_mode(DUMMY_MODE_INTERRUPTABLE);
 *          dummy_arm_enable();
 *          dummy_arm_home();
 *          dummy_arm_move_joints(10, 0, 90, 0, 0, 0, 30);
 *      }
 *
 ******************************************************************************
 */

#ifndef USB_HOST_CDC_RASC_GUIDE_H_
#define USB_HOST_CDC_RASC_GUIDE_H_
/* 本文件仅作配置说明，无实际代码 */
#endif
