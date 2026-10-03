#ifndef TEST_HELPER_HPP
#define TEST_HELPER_HPP

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
 * @brief EXPECT 断言：失败仅计数，继续执行后续测试
 * @param condition 判断表达式
 * @param case_name 字符串字面量，测试用例名称
 * @note 续行符\必须是行最后一个字符，后面不能有空格
 */
#define EXPECT(condition, case_name) \
do{ \
    bool const cond_val = static_cast<bool>(condition); \
    expect_impl(cond_val, (case_name), __FILE__, __LINE__); \
}while(0)

/**
 * @brief ASSERT 断言：失败直接return退出当前测试函数
 * @param condition 判断表达式
 * @param case_name 字符串字面量，测试用例名称
 */
#define ASSERT(condition, case_name) \
do{ \
    bool const cond_val = static_cast<bool>(condition); \
    if(!cond_val){ \
        expect_impl(false, case_name, __FILE__, __LINE__); \
        return false; \
    } \
}while(0)

#endif // TEST_HELPER_HPP