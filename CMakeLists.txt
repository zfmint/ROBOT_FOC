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
# robot_hal_core：生产核心静态库，HAL抽象层 + COMMON，全平台构建
# 不含Mock模拟驱动；STM32固件、PC测试均依赖此库
##############################################################################
add_library(robot_hal_core STATIC
    # Src/HAL：HAL抽象层实现文件
    Src/HAL/peripheral.cpp
    Src/HAL/gpio.cpp
    # Src/HAL 后续新增HAL cpp写在这里

    # Src/COMMON 公共组件实现
    # Src/COMMON/xxx.cpp
)

## robot_hal_core编译选项：生产库关闭异常、RTTI，适配STM32嵌入式
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(robot_hal_core PRIVATE
        /utf-8
        /W4
        /permissive-
        /EHs-c-
        /GR-
    )
else()
    target_compile_options(robot_hal_core PRIVATE
        -Wall -Wextra -Wpedantic
        -fno-exceptions
        -fno-rtti
    )
endif()

target_include_directories(robot_hal_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/Inc
)

##############################################################################
# robot_hal_mock：PC仿真驱动库【仅本机PC编译，交叉编译跳过】
# MockGpio / MockUart 模拟外设实现，仅供单元测试使用，禁止固件链接
##############################################################################
if(NOT CMAKE_CROSSCOMPILING)
    add_library(robot_hal_mock STATIC
        Src/DRIVERS/mock_gpio.cpp
        # 后续新增Mock实现全部放这里
    )

    ## Mock库编译选项：同core，关闭异常RTTI，行为对齐生产库
    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        target_compile_options(robot_hal_mock PRIVATE
            /utf-8
            /W4
            /permissive-
            /EHs-c-
            /GR-
        )
    else()
        target_compile_options(robot_hal_mock PRIVATE
            -Wall -Wextra -Wpedantic
            -fno-exceptions
            -fno-rtti
        )
    endif()

    target_link_libraries(robot_hal_mock PUBLIC robot_hal_core)
endif()

##############################################################################
# pc_unit_test：PC平台单元测试可执行程序
# @brief 仅PC本机编译时构建；交叉编译STM32固件时跳过，不编译PC测试代码
# @details
#  测试程序包含main函数、控制台std输出，不可编译进单片机固件镜像。
#  依赖robot_hal_core + robot_hal_mock，加载HAL抽象层与Mock模拟驱动用于单元验证。
# @note
#  PRIVATE头文件路径仅作用于当前测试目标，不会向外传递到其它依赖目标；
#  生产代码无法访问Test/Inc下测试专用头文件，实现测试与业务代码隔离。
##############################################################################
if(NOT CMAKE_CROSSCOMPILING)
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

    ## @brief 链接核心库 + mock仿真库
    target_link_libraries(pc_unit_test
        PRIVATE
            robot_hal_core
            robot_hal_mock
    )

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