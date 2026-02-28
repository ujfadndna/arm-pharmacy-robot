/**
 * @file    cabinet_demo.h
 * @brief   智能药柜整理功能演示接口
 */

#ifndef CABINET_DEMO_H_
#define CABINET_DEMO_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行智能药柜整理演示
 *
 * 演示流程:
 * 1. 初始化药品数据库和药柜状态
 * 2. 模拟接收MaixCam2的TAG消息
 * 3. 生成药柜状态字符串
 * 4. 解析LLM返回的动作序列
 * 5. 执行动作
 */
void cabinet_demo_run(void);

#ifdef __cplusplus
}
#endif

#endif /* CABINET_DEMO_H_ */
