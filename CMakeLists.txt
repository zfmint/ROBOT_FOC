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

##############################################################################
# PC本机编译编译选项配置块
# NOT CMAKE_CROSSCOMPILING：CMake内置变量
#  - Windows/Linux本机编译：CMAKE_CROSSCOMPILING=OFF，进入本分支
#  - ARM工具链交叉编译STM32：CMAKE_CROSSCOMPILING=ON，跳过本块
# 参考 CODING_STANDARD.md：开启严格警告，尽早发现代码隐患
##############################################################################
if(NOT CMAKE_CROSSCOMPILING)
    # CMAKE_CXX_COMPILER_ID：获取当前C++编译器标识
    # STREQUAL "MSVC"：判断当前编译器是否微软MSVC(Visual Studio编译器)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        # add_compile_options：添加全局编译选项，对后续所有目标生效
        # /utf-8：源码文件使用UTF‑8编码，避免中文注释乱码
        # /W4：MSVC最高警告等级(生产项目建议W4，对应规范要求)
        # /permissive‑：关闭MSVC非标准扩展，严格遵循C++标准
        add_compile_options(/utf-8 /W4 /permissive-)
    else()
        # GCC / Clang 编译器（MinGW、Linux GCC）
        # -Wall：开启基础警告；-Wextra：开启额外扩展警告；-Wpedantic：严格标准合规检查
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
endif()

##############################################################################
# robot_hal：生产静态库目标，HAL抽象层 + 设备驱动层核心业务代码
# 注意：Header‑only模板头文件（例如 byte_ring_buffer.hpp）
#       不需要写在add_library源文件列表，仅需要放置在Inc目录，通过include路径暴露
#       模板实现全部写在.hpp头文件，无对应的.cpp实现文件
##############################################################################
# add_library：创建库目标；STATIC代表静态库（*.lib / *.a）
add_library(robot_hal STATIC
    # Src/HAL：HAL抽象层实现文件
    Src/HAL/peripheral.cpp
    Src/HAL/gpio.cpp
    # Src/DRIVERS：模拟驱动实现文件(MockGpio)
    Src/DRIVERS/mock_gpio.cpp
)

# target_include_directories：为目标设置头文件搜索目录
# PUBLIC：
#   1. robot_hal库本身编译时，可以搜索该目录
#   2. 任何target_link_libraries链接robot_hal的目标，编译期自动继承该头文件搜索路径
# 生产库robot_hal只对外暴露项目Inc；test/Inc是测试专用头文件，禁止注入生产库
target_include_directories(robot_hal
    PUBLIC
        ${PROJECT_SOURCE_DIR}/Inc
)
# 变量说明：
# ${PROJECT_SOURCE_DIR}：cmake内置变量，代表项目根目录（CMakeLists.txt所在目录）

##############################################################################
# pc_test_peripheral：PC平台单元测试可执行程序
# 仅本机PC编译时构建；交叉编译STM32固件时跳过，不编译PC测试代码
# 原因：测试代码包含main函数、控制台输出，不能编译进单片机固件
##############################################################################
if(NOT CMAKE_CROSSCOMPILING)
    # add_executable：生成可执行程序pc_test_peripheral
    add_executable(pc_test_peripheral
        # 测试程序入口main
        test/pc_test_main.cpp
        # 虚拟外设基础测试
        test/Src/test_dummy_periph.cpp
        # MockGpio单元测试
        test/Src/test_mock_gpio.cpp
        # 环形缓冲区单元测试
        test/Src/test_byte_ring_buffer.cpp
    )

    # PRIVATE：头文件路径仅当前pc_test_peripheral编译时生效
    # 不会传递给任何链接pc_test_peripheral的其他目标
    # test/Inc存放测试辅助头文件，只给测试程序使用，不对外扩散
    target_include_directories(pc_test_peripheral
        PRIVATE
            ${PROJECT_SOURCE_DIR}/test/Inc
    )

    # target_link_libraries：库链接关系配置
    # PRIVATE：pc_test_peripheral链接robot_hal；
    #          如果别的目标链接pc_test_peripheral，不会自动链接robot_hal
    target_link_libraries(pc_test_peripheral PRIVATE robot_hal)
endif()