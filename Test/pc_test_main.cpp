/**
 * @file pc_test_main.cpp
 * @brief PC端单元测试程序入口主函数
 * @details
 * 执行整套项目单元测试套件，支持命令行参数控制输出模式。
 *
 * ## 命令行选项
 * | 参数 | 说明 |
 * |------|------|
 * | --verbose | 开启详细输出，打印全部[PASS]/[FAIL]测试用例；默认仅打印失败用例 |
 *
 * ## 输出行为
 * 1. 默认模式：仅输出失败用例信息，包含用例名称、源码文件、行号；成功用例静默不输出。
 * 2. --verbose模式：打印全部成功与失败用例信息。
 * 3. 程序结束输出统计：总用例数、成功数量、失败数量。
 * 4. 返回值：0全部测试通过；非0存在测试失败，适合CI自动化集成。
 *
 * @note
 * 1. 本程序仅用于PC平台单元测试，**不编译进MCU固件**。
 * 2. 测试断言宏与全局计数器定义在 test_helper.hpp。
 * 3. 所有测试函数在此统一调度执行，不做并行执行，顺序串行运行。
 *
 * @retval EXIT_SUCCESS(0) 全部测试用例执行通过
 * @retval EXIT_FAILURE(1) 存在至少一条测试用例失败
 */

#include <cstdlib>
#include <iostream>
#include <cstring>

#include "test_helper.hpp"

/// @brief 测试入口声明
void test_dummy_peripheral();
void test_mock_gpio();
void test_byte_ring_buffer();
void test_mock_uart();
void test_error_code();
void test_positional_pid();
void test_positional_pid_closed();
void test_incremental_pid();
void test_incremental_pid_closed();
void test_pi_controller();
void test_pi_controller_closed();
void test_sliding_average();
void test_encoder_speed();
void test_encoder_speed_closed();
void test_foc_transform();
void test_voltage_limit();
void test_svpwm();
void test_current_loop();

// 改动：增加 argc, argv 参数，用于读取命令行选项
int main(int argc, char* argv[])
{
    // 解析 --verbose 开关
    for(int i = 1; i < argc; ++i)
    {
        if(std::strcmp(argv[i], "--verbose") == 0)
        {
            test_verbose = true;
        }
    }

    std::cout << "\n========== Start Unit Test ==========\n\n";

    // 全部原有测试调用，一行都不用改
    test_dummy_peripheral();
    test_mock_gpio();
    test_byte_ring_buffer();
    test_mock_uart();
    test_error_code();
    test_positional_pid();
    test_positional_pid_closed();
    test_incremental_pid();
    test_incremental_pid_closed();
    test_pi_controller();
    test_pi_controller_closed();
    test_sliding_average();
    test_encoder_speed();
    test_encoder_speed_closed();
    test_foc_transform();
    test_voltage_limit();
    test_svpwm();
    test_current_loop();

    std::cout << "\n====================================\n";
    const auto total = test_success_count + test_failure_count;
    std::cout << "Total cases:  " << total << '\n';
    std::cout << "Success:      " << test_success_count << '\n';
    std::cout << "Failed:       " << test_failure_count << '\n';
    std::cout << "====================================\n";

    if (test_failure_count != 0U)
    {
        std::cerr << "\nSome test cases failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "\n✅ All tests passed.\n";
    return EXIT_SUCCESS;
}
