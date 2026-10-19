#include <cstdlib>
#include <iostream>

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

int main()
{
    std::cout << "\n========== Start Unit Test ==========\n\n";
    test_dummy_peripheral();
    test_mock_gpio();
    test_byte_ring_buffer();
    test_mock_uart();
    test_error_code();
    test_positional_pid();
    test_positional_pid_closed();
    test_incremental_pid();
    test_incremental_pid_closed();

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
