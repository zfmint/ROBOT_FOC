#include "test_helper.hpp"

// 【声明所有测试套件函数原型】
// 新增测试套件只需要在这里增加函数声明
bool test_dummy_peripheral();
bool test_mock_gpio();
// bool test_mock_uart();
// bool test_mock_spi();

int main()
{
    std::cout << "\n========== Start Unit Test ==========\n\n";

    // ========== 测试套件注册列表（仅在此添加套件调用，main其余代码不动） ==========
    test_dummy_peripheral();
    test_mock_gpio();
    // test_mock_uart();
    // test_mock_spi();

    // ========== 下面统计逻辑固定，永远不需要修改 ==========
    std::cout << "\n====================================\n";
    std::cout << "Total test failed: " << test_fail_cnt << "\n";
    if (test_fail_cnt > 0)
    {
        std::cerr << "❌ Some test cases failed!\n";
        return EXIT_FAILURE;
    }
    std::cout << "✅ All test passed!\n";
    return EXIT_SUCCESS;
}