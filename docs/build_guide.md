# ROBOT_FOC CMake构建说明

## 一、 文档信息

##### 适用范围：CMakeLists.txt
##### 作者：zfmint
##### 最后更新日期 2026.10.10
##### 相关文档：编码规范、README、架构说明文档

## 二、 工程概述

1. 本项目采用CMake >= 3.16 构建，当前支持两套编译环境。
 - PC本机编译：\
 构建核心库 + Mock仿真库 + PC单元测试，此处用于模块编译和测试
 - STM32F4交叉编译\
 仅构建核心库`robot_hal_core`,Mock仿真库与test测试跳过，不会编译进单片机。
2. 构建目标
 - `robot_hal_core`\
 静态库、全平台、HAL + COMMON、进入固件
 - `robot_hal_mock`\
 静态库、仅PC、Mock文件、不进入固件
 - `pc_unit_test`\
 PC 测试可执行文件，仅 PC 构建，不进入固件
3. 依赖关系
 依赖箭头指向被依赖方
 ```
 pc_unit_test -> robot_hal_mock -> robot_hal_core
 ```
 - 约束关系\
 STM固件工程只链接`robot_hal_core`,若固件代码`include`了Mock，链接阶段会报错
 - C++标准\
 使用C++17，禁止编译器扩展\
 生产库关闭异常、关闭RTTI\
 PC测试启用异常，支持控制台输出

## 三、构建目标

1. `robot_hal_core` （生产核心静态库，支持全平台）
 - 作用域\
 HAL抽象层 + COMMON公共组件，作为全项目公用的底层依赖
 - 配置项\
 当前源文件：`Src/HAL/*.cpp`；后续新增公共组件实现可放入 `Src/COMMON/`\
 头文件目录：`Inc`\
 MSVC编译选项：`/utf-8`、`/W4`、`/permissive-`、`/EHs-c-`（关异常）、`/GR-`（关 RTTI）\
 GCC/ARM-GCC: `-Wall -Wextra -Wpedantic`、`-fno-exceptions`、`-fno-rtti`
 - 约束条件\
 生产库不得引入 `Test/` 目录任何头文件；模板类 `ByteRingBuffer`全头文件实现，不加入 `add_library` 源文件列表
2. `robot_hal_mock`（Mock 仿真库，仅 PC）
 - 作用域\
 `MockGpio` / `MockUart` 仿真驱动实现，仅供 `pc_unit_test` 链接使用
 - 配置项\
 源文件：`Src/DRIVERS/mock_gpio.cpp`；`MockUart` 为头文件模板实现，不存在单独的 `.cpp` 文件 \
 MSVC编译选项：`/utf-8`、`/W4`、`/permissive-`、`/EHs-c-`（关异常）、`/GR-`（关 RTTI）\
 GCC: `-Wall -Wextra -Wpedantic`、`-fno-exceptions`、`-fno-rtti`
 链接依赖：PUBLIC 链接 `robot_hal_core`
 - 约束条件\
 头文件层面对外可见；但固件未链接 mock 库
3. `pc_unit_test`（PC 单元测试可执行程序）
 - 作用域\
 加载 `robot_hal_core` + `robot_hal_mock` 运行全部单元测试
 - 配置项\
 源文件：`Test/pc_test_main.cpp` + `Test/Src/*.cpp` 全部测试用例 \
 私有目录：`Test/Inc`（PRIVATE，仅测试程序可见，生产代码无法包含测试头）
 MSVC编译选项：`/EHsc`（启用异常）\
 GCC: `-Wall -Wextra -Wpedantic`（不关闭异常、RTTI）
 链接依赖：PRIVATE 链接 `robot_hal_core` + `robot_hal_mock`
 输出程序：Visual Studio 多配置生成器为 `build/bin/Debug/pc_unit_test.exe`；单配置生成器通常为 `build/bin/pc_unit_test`
 - 约束条件\
 `Test/Inc` 使用 PRIVATE，不会传递给 `robot_hal_core`；生产代码永远无法 `#include` 测试辅助头文件

## 四、PC编译步骤

1. 前置条件
 - CMake ≥3.16
 - C++17 编译器：MSVC 2019+ / GCC 8+ / Clang
2. 编译步骤
 ```bash
 # 1. 在项目根目录配置
 cmake -S . -B build
 # 2. 编译全部 PC 目标（core + mock + pc_unit_test）
 cmake --build build --config Debug --clean-first
 # 3. 运行单元测试程序（Visual Studio 多配置生成器）
 # Windows PowerShell
 .\build\bin\Debug\pc_unit_test.exe
 # 单配置生成器通常使用：./build/bin/pc_unit_test
 ```
3. Windows 注意事项
 - MSVC 全局默认注入 `/EHsc`，脚本在 `if(MSVC)` 块中先清除全局 EH 标志，再由每个目标单独管控异常开关；
 - 若修改 `CMakeLists.txt` 后出现缓存或生成器状态异常，优先重新配置 build 目录：
 ```powershell
 cmake -S . -B build
 cmake --build build --config Debug --clean-first
 ```

## 五、交叉编译步骤

1. 编译约束\
`CMAKE_CROSSCOMPILING=TRUE`，此时 `robot_hal_mock` 与 `pc_unit_test` **均不会被构建**，仅编译 `robot_hal_core` 静态库，供上层固件工程链接。
2. 编译步骤
 ```bash
 # 使用 ARM‑GCC 工具链文件
 # 当前仓库尚未提交 STM32F4 工具链文件；接入阶段三工具链后再执行：
 cmake -S . -B build-stm32 -DCMAKE_TOOLCHAIN_FILE=<stm32f4_toolchain.cmake>
 cmake --build build-stm32
 ```
3. 交叉编译注意事项\
 - 交叉编译时 Test 目录全部测试源码不参与编译；
 - `robot_hal_core` 强制关闭异常、RTTI，与 STM32 运行环境匹配；
 - 本 CMake 脚本只负责构建 HAL 静态库；STM32 启动文件、链接脚本、主固件工程由上层工程管理；
 - 阶段三接入真实驱动时，新增 `robot_hal_stm32` 目标（仅交叉编译构建），固件链接 `robot_hal_core + robot_hal_stm32`。

## 六、关键编译内容

1. 全局C++设置
 ```cmake
 set(CMAKE_CXX_STANDARD 17)
 set(CMAKE_CXX_STANDARD_REQUIRED ON)
 set(CMAKE_CXX_EXTENSIONS OFF)
 ```
 强制 C++17，禁用编译器扩展，保证跨编译器行为一致。
2. MSCV异常标志清理
 ```cmake
 if(MSVC)
    string(REGEX REPLACE "(^| )[/-]EH[^ ]*" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
 endif()
 ```
 MSVC 默认全局注入 `/EHsc`；此处清除全局异常选项，每个目标单独管控异常开关，防止生产库意外启用异常。


