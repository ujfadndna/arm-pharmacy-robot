/**
 * @file    test_llm_parser.c
 * @brief   PC端JSON解析器测试程序
 *
 * 编译命令 (Windows):
 *   gcc -o test_parser.exe test_llm_parser.c ../src/llm_action.c -I../src
 *
 * 运行:
 *   test_parser.exe
 */

#include <stdio.h>
#include <string.h>
#include "llm_action.h"

/* 测试用例 */
typedef struct {
    const char *name;
    const char *json;
    int expected_count;
} test_case_t;

static test_case_t test_cases[] = {
    {
        "基本测试 - 完整动作序列",
        "[{\"action\": \"move\", \"x\": 120, \"y\": 80, \"z\": 40},"
        "{\"action\": \"grab\"},"
        "{\"action\": \"speak\", \"text\": \"Done\"}]",
        3
    },
    {
        "单个move动作",
        "[{\"action\": \"move\", \"x\": 100.5, \"y\": -50.3, \"z\": 200}]",
        1
    },
    {
        "带release的序列",
        "[{\"action\": \"move\", \"x\": 0, \"y\": 0, \"z\": 100},"
        "{\"action\": \"release\"}]",
        2
    },
    {
        "空数组",
        "[]",
        0
    },
    {
        "DeepSeek实际返回格式",
        "[  {\"action\": \"move\", \"x\": 120, \"y\": 80, \"z\": 40},\n"
        "   {\"action\": \"grab\"},\n"
        "   {\"action\": \"speak\", \"text\": \"已取药\"}\n]",
        3
    },
    {
        "带scan_qr的序列",
        "[{\"action\": \"scan_qr\"}, {\"action\": \"speak\", \"text\": \"Scanned\"}]",
        2
    },
    {NULL, NULL, 0}  /* 结束标记 */
};

static void print_action(int idx, const llm_action_t *a)
{
    printf("  [%d] %s", idx, llm_action_type_str(a->type));

    switch (a->type) {
        case ACTION_MOVE:
            printf(" x=%.1f y=%.1f z=%.1f",
                   a->params.move.x, a->params.move.y, a->params.move.z);
            break;
        case ACTION_SPEAK:
            printf(" text=\"%s\"", a->params.speak.text);
            break;
        default:
            break;
    }
    printf("\n");
}

static int run_test(const test_case_t *tc)
{
    printf("\n=== %s ===\n", tc->name);
    printf("Input: %s\n", tc->json);

    llm_action_sequence_t seq;
    int ret = llm_parse_json(tc->json, &seq);

    printf("Result: ret=%d, count=%d (expected=%d)\n",
           ret, seq.count, tc->expected_count);

    /* 打印解析结果 */
    for (int i = 0; i < seq.count; i++) {
        print_action(i, &seq.actions[i]);
    }

    /* 验证 */
    int pass = 1;
    if (tc->expected_count == 0) {
        if (ret == 0 || seq.count != 0) {
            pass = 0;
        }
    } else {
        if (ret != 0 || seq.count != tc->expected_count) {
            pass = 0;
        }
    }

    printf(">>> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

int main(void)
{
    printf("========================================\n");
    printf("    LLM JSON Parser Test Suite\n");
    printf("========================================\n");

    int total = 0, passed = 0;

    for (int i = 0; test_cases[i].name != NULL; i++) {
        total++;
        if (run_test(&test_cases[i])) {
            passed++;
        }
    }

    printf("\n========================================\n");
    printf("    Results: %d/%d passed\n", passed, total);
    printf("========================================\n");

    return (passed == total) ? 0 : 1;
}
