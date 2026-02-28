/**
 * @file    llm_test.h
 * @brief   LLM调用测试接口
 */

#ifndef LLM_TEST_H_
#define LLM_TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 测试LLM调用 (阻塞)
 */
void test_llm_call(void);

/**
 * @brief 创建LLM测试任务 (非阻塞)
 */
void create_llm_test_task(void);

#ifdef __cplusplus
}
#endif

#endif /* LLM_TEST_H_ */
