/**
 * @file    w800_cmd_handler.h
 * @brief   W800 TCP透传 JSON 命令处理
 */

#ifndef W800_CMD_HANDLER_H_
#define W800_CMD_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 处理一行 JSON 命令（不含结尾 \r\n）
 * @param json_line JSON 字符串
 */
void w800_cmd_handle(const char *json_line);

#ifdef __cplusplus
}
#endif

#endif /* W800_CMD_HANDLER_H_ */
