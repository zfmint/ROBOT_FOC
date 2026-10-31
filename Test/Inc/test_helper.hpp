/**
 * @file test_helper.hpp
 * @brief 极简单元测试断言框架，用于PC端单元测试
 * @details
 *  提供两组断言宏：ROBOT_FOC_TEST_EXPECT / ROBOT_FOC_TEST_ASSERT。
 *  - ROBOT_FOC_TEST_EXPECT：断言失败仅增加失败计数，继续执行本测试函数剩余代码。
 *  - ROBOT_FOC_TEST_ASSERT：断言失败计数+1，并立即return退出当前测试函数。
 *  全局维护失败计数器 test_failure_count，支持多测试源文件共同统计。
 *  仅用于PC单元测试，**不编译进MCU固件**。
 * @author Robot_FOC Project
 * @note
 *  1. 宏定义续行符 \ 必须放在行末尾，后面不能存在空格。
 *  2. 断言第二个参数必须传入字符串字面量，作为用例名称。
 *  3. inline全局变量，解决多文件include造成重复定义链接错误。
 */

#ifndef ROBOT_FOC_TEST_TEST_HELPER_HPP
#define ROBOT_FOC_TEST_TEST_HELPER_HPP

#include <cstddef>
#include <cstdlib>
#include <iostream>

// 全局测试失败计数，inline保证多文件包含不重复定义
inline std::size_t test_failure_count{0U};

/**
 * @brief 测试断言底层实现函数
 * @param condition 判定条件真假
 * @param case_name 测试用例名字符串
 * @param file 源文件名 __FILE__
 * @param line 代码行号 __LINE__
 */
inline void expect_impl(
    const bool condition,
    const char* case_name,
    const char* file,
    const int line) noexcept
{
    if (!condition)
    {
        ++test_failure_count;
        std::cerr << "[FAIL] " << case_name << " | " << file << ':' << line << '\n';
        return;
    }
    std::cout << "[PASS] " << case_name << '\n';
}

/**
 * @brief ROBOT_FOC_TEST_EXPECT 断言：失败仅计数，继续执行后续测试
 * @param condition 判断表达式
 * @param case_name 字符串字面量，测试用例名称
 * @note 续行符\必须是行最后一个字符，后面不能有空格
 */
#define ROBOT_FOC_TEST_EXPECT(condition, case_name) \
    do                                            \
    {                                             \
        const bool cond_val = static_cast<bool>(condition); \
        expect_impl(cond_val, (case_name), __FILE__, __LINE__); \
    } while (false)

/**
 * @brief ROBOT_FOC_TEST_ASSERT 断言：失败直接return退出当前测试函数
 * @param condition 判断表达式
 * @param case_name 字符串字面量，测试用例名称
 */
#define ROBOT_FOC_TEST_ASSERT(condition, case_name) \
    do                                            \
    {                                             \
        const bool cond_val = static_cast<bool>(condition); \
        if (!cond_val)                            \
        {                                         \
            expect_impl(false, (case_name), __FILE__, __LINE__); \
            return;                               \
        }                                         \
    } while (false)

#endif // ROBOT_FOC_TEST_TEST_HELPER_HPP

namespace
{
    constexpr float kTolerance{1.0e-4F};

    [[nodiscard]] bool nearly_equal(float lhs, float rhs, float tolerance = kTolerance) noexcept
    {
        return std::fabs(lhs - rhs) <= tolerance;
    }
}

