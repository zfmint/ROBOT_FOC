#include <cstdlib>
#include <iostream>

#include "test_helper.hpp"

/// @brief 新测试在此处声明
/* 
bool test_dummy_peripheral();
bool test_mock_gpio();
bool test_byte_ring_buffer();
bool test_mock_uart();
bool test_error_code();*/
bool test_positional_pid();
bool test_positional_pid_closed();
bool test_incremental_pid();
bool test_incremental_pid_closed();

int main()
{
    std::cout << "\n========== Start Unit Test ==========\n\n";
    /// 在此进行函数运行
    /* 
    (void)test_dummy_peripheral();
    (void)test_mock_gpio();
    (void)test_byte_ring_buffer();
    (void)test_mock_uart();
    (void)test_error_code();*/
    (void)test_positional_pid();
    (void)test_positional_pid_closed();
    (void)test_incremental_pid();
    (void)test_incremental_pid_closed();

    std::cout << "\n====================================\n";
    std::cout << "Total test failed: " << test_failure_count << '\n';

    if (test_failure_count != 0U)
    {
        std::cerr << "Some test cases failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All tests passed.\n";
    return EXIT_SUCCESS;
}
