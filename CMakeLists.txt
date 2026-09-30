#制定Cmake最低版本要求，
cmake_minimum_required(VERSION 3.16)
#项目名称，指定使用语言
project(Robot_FOC LANGUAGES CXX)
#设置C++标准为C++17，
set(CMAKE_CXX_STANDARD 17)
#强制要求C++版本，不允许回退
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#关闭编译器扩展
set(CMAKE_CXX_EXTENSIONS OFF)

#创建静态库
#把HAL层的peripheral.cpp编译进这个库
#后续新增HAL外设cpp往此追加
add_library(robot_hal STATIC
    Src/HAL/peripheral.cpp
)

#将Inc目录加入头文件搜索路径
#代码内可以直接写文件名，不用写相对路径
target_include_directories(robot_hal
    PUBLIC
        ${PROJECT_SOURCE_DIR}/Inc
)

# 编译器为“MSVC”
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/utf-8)
    target_compile_options(robot_hal PRIVATE
        #W4为最高警告等级，
        /W4
        #警告只提示，不阻止编译
        /WX-
    )
else()
    target_compile_options(robot_hal PRIVATE
        #开启常见警告
        -Wall
        #开启一批额外警告
        -Wextra
    )
endif()
#判断：如果不是交叉编译（在当前PC编译，不是编译STM32固件）
#交叉编译STM32时，CMAKE_CROSSCOMPILING会为true，自动跳过以下测试代码
if(NOT CMAKE_CROSSCOMPILING)
    #生成PC端测试可执行程序
    #源码为test目录下的pc_test_main.cpp
    add_executable(pc_test_peripheral
        test/pc_test_main.cpp
    )
    #让测试程序链接robot_hal静态库，使用HAL层的Peripheral基类代码
    target_link_libraries(pc_test_peripheral PRIVATE robot_hal)
endif()