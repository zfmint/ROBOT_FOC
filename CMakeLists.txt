# cmake_minimum_required：指定本工程要求的CMake最低版本
# 3.16是比较稳妥基线，支持C++17、FetchContent、交叉编译相关特性
cmake_minimum_required(VERSION 3.16)

# project：定义项目名称、支持的编程语言
# LANGUAGES CXX：本项目只使用C++，不启用C语言编译器
# 项目名：Robot_FOC，用于后续变量、工程文件名
# 本工程双目标：①PC本机编译单元测试 ②交叉编译STM32F4嵌入式固件
project(Robot_FOC LANGUAGES CXX)

# 设置C++语言标准：使用C++17
set(CMAKE_CXX_STANDARD 17)
# CMAKE_CXX_STANDARD_REQUIRED ON：强制要求C++17，不自动回退到更低C++版本
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# CMAKE_CXX_EXTENSIONS OFF：关闭编译器扩展语法，使用标准C++17，提升跨编译器兼容性
set(CMAKE_CXX_EXTENSIONS OFF)

# Visual Studio生成器默认通过CMAKE_CXX_FLAGS注入/EHsc。
# 先移除全局异常选项，再由各目标分别选择异常模型，避免/EHsc与/EHs-c-互相覆盖。
if(MSVC)
    string(REGEX REPLACE "(^| )[/-]EH[^ ]*" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

##############################################################################
# robot_hal：生产静态库目标，HAL抽象层 + 设备驱动层核心业务代码
# @brief 生产库，用于STM32固件，禁用C++异常、RTTI
# @details
#  Header‑only模板头文件（byte_ring_buffer.hpp、mock_uart.hpp）
#  不需要写在add_library源文件列表，仅放置在Inc目录，通过include路径暴露
#  模板实现全部写在.hpp头文件，无对应的.cpp实现文件
##############################################################################
add_library(robot_hal STATIC
    # Src/HAL：HAL抽象层实现文件
    Src/HAL/peripheral.cpp
    Src/HAL/gpio.cpp
    # Src/DRIVERS：模拟驱动实现文件(MockGpio)

)

## @brief robot_hal编译选项：生产库关闭异常、RTTI，适配STM32嵌入式
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # robot_hal独立编译选项，关闭C++异常+RTTI
    target_compile_options(robot_hal PRIVATE
        /utf-8
        /W4
        /permissive-
        /EHs-c-
        /GR-
    )
else()
    target_compile_options(robot_hal PRIVATE
        -Wall -Wextra -Wpedantic
        -fno-exceptions
        -fno-rtti
    )
endif()

target_include_directories(robot_hal
    PUBLIC
        ${PROJECT_SOURCE_DIR}/Inc
)

##############################################################################
# pc_unit_test：PC平台单元测试可执行程序
# @brief 仅PC本机编译时构建；交叉编译STM32固件时跳过，不编译PC测试代码
# @details
#  测试程序包含main函数、控制台std输出，不可编译进单片机固件镜像。
#  依赖robot_hal静态库，加载HAL抽象层与Mock模拟驱动用于单元验证。
# @note
#  PRIVATE头文件路径仅作用于当前测试目标，不会向外传递到其它依赖目标；
#  生产代码无法访问Test/Inc下测试专用头文件，实现测试与业务代码隔离。
##############################################################################
if(NOT CMAKE_CROSSCOMPILING)
    # add_executable：生成可执行程序pc_unit_test
    add_executable(pc_unit_test
        # 测试程序入口main
        Test/pc_test_main.cpp
        # 虚拟外设基础测试
        Test/Src/test_dummy_periph.cpp
        # MockGpio单元测试
        Test/Src/test_mock_gpio.cpp
        # 环形缓冲区单元测试
        Test/Src/test_byte_ring_buffer.cpp
        # 串口单元测试
        Test/Src/test_mock_uart.cpp
        # 错误码单元测试
        Test/Src/test_error_code.cpp
    )

    ## @brief 测试目标私有头文件搜索目录
    ## @note Test/Inc存放测试辅助头文件，仅pc_unit_test可见，禁止污染生产库
    target_include_directories(pc_unit_test
        PRIVATE
            ${PROJECT_SOURCE_DIR}/Test/Inc
    )

    ## @brief 链接生产静态库robot_hal
    ## @note PRIVATE：仅当前测试目标使用robot_hal，不向外传递依赖
    target_link_libraries(pc_unit_test PRIVATE robot_hal)

    ## @brief 设置可执行产物输出目录
    ## @details 编译生成的pc_unit_test.exe放置于 build/bin，方便统一管理二进制
    set_target_properties(pc_unit_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )

    ## @brief PC测试程序：MSVC开启异常，支持std::cout控制台输出
    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        target_compile_options(pc_unit_test PRIVATE
            /utf-8
            /W4
            /permissive-
            /EHsc
        )
    else()
        target_compile_options(pc_unit_test PRIVATE
            -Wall -Wextra -Wpedantic
        )
    endif()
endif()
