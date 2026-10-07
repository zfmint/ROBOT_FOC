#include <cstdlib>
#include <iostream>

#include "test_helper.hpp"

bool test_dummy_peripheral();
bool test_mock_gpio();
bool test_byte_ring_buffer();
bool test_mock_uart();

int main()
{
    std::cout << "\n========== Start Unit Test ==========\n\n";

    (void)test_dummy_peripheral();
    (void)test_mock_gpio();
    (void)test_byte_ring_buffer();
    (void)test_mock_uart();

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
