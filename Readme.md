# Robot_FOC

基于 STM32F4 与 FreeRTOS 的机器人移动底盘嵌入式控制系统。项目采用“PC 端 C++ 先实现和验证，再移植到 STM32”的开发流程，目标覆盖 FOC 电机控制、差速底盘运动学、车载档位交互、故障保护、外设驱动封装和实时多任务调度。

当前处于第一阶段“嵌入式 C++ 架构筑基”。Day1～Day6 已完成，Day7 功能与测试已实现，正在进行编码规范收尾。Python 自动化测试暂缓，目前使用 PC C++ 单元测试验证基础组件。

## 开发原则

- 使用 C++17，生产代码关闭异常和 RTTI。
- 硬件外设直接或间接继承 `Peripheral`，统一使用 `init()` / `deinit()` 生命周期接口。
- 外设对象禁止复制和移动，避免硬件资源所有权被隐式转移。
- 固定容量容器，不使用动态内存。
- 中断只采集原始数据，复杂处理交给任务执行。
- 算法和业务逻辑必须先通过 PC 测试，再移植到 STM32。
- Python、pybind11 和 pytest 尚未接入，不属于当前构建流程。

## 目标软件架构

```text
RTOS 调度层
    └── FreeRTOS 任务、队列、信号量和任务通知
算法与业务层
    └── PID、滤波、FOC、运动学、状态机和故障诊断
设备驱动层
    └── MockGpio、MockUart，以及后续 STM32 外设实现
HAL 硬件抽象层
    └── Peripheral、Gpio、Uart、ByteRingBuffer 和通用类型
```

当前已实现的主要关系：

```text
Peripheral
├── Gpio
│   └── MockGpio
└── Uart<RxBufferCapacity>
    └── MockUart<RxBufferCapacity, TxBufferCapacity>

Uart 和 MockUart 组合使用 ByteRingBuffer 作为固定容量收发缓冲区。
```

## 当前功能

- `Peripheral` 统一生命周期和初始化状态管理。
- `Gpio` 抽象接口及 PC 端 `MockGpio` 实现。
- `Uart` 模板抽象接口及 PC 端 `MockUart` 实现。
- 固定容量 `ByteRingBuffer`，支持 FIFO、满/空判断、环绕、窥视和清空。
- 通用 `ErrorCode` 和基础物理量单位类型。
- 使用 `static_assert` 检查关键类型不可复制、不可移动。
- GPIO 通用行为测试只依赖 `Gpio&`，便于后续替换为 STM32 实现。
- MSVC 与 GCC/Clang 严格警告配置；生产库关闭异常和 RTTI。

## 目录结构

```text
Robot_FOC/
├── Inc/
│   ├── COMMON/              # 错误码和基础单位类型
│   ├── DRIVERS/             # PC Mock 设备驱动
│   └── HAL/                 # 外设抽象接口和固定容量工具
├── Src/
│   ├── DRIVERS/             # Mock 驱动实现
│   └── HAL/                 # HAL 非模板实现
├── test/
│   ├── Inc/                 # PC 测试辅助接口
│   ├── Src/                 # C++ 单元测试
│   ├── pc_test_main.cpp     # PC 测试入口
│   └── pybind/              # 预留目录，当前未接入构建
├── CMakeLists.txt
├── CODING_STANDARD.md
└── Readme.md
```

`build/` 是本地构建输出目录，不提交到 Git。

## 构建环境

- CMake 3.16 或更高版本
- 支持 C++17 的编译器
- Windows 推荐 Visual Studio 2022 / MSVC
- GCC 或 Clang 可用于额外的跨编译器检查

## PC 构建与测试

在项目根目录执行：

```powershell
cmake -S . -B build
cmake --build build --config Debug --clean-first
.\build\bin\Debug\pc_unit_test.exe
```

使用 Visual Studio 多配置生成器时，测试程序位于 `build/bin/Debug/`。使用单配置生成器时，程序通常位于 `build/bin/`。

测试程序会执行全部用例并汇总失败数量：全部通过时返回 `0`，任何断言失败时返回非零退出码。

当前测试覆盖：

- `Peripheral` 初始化、反初始化、重复调用和实例状态隔离。
- `MockGpio` 配置、模式、电平、生命周期和基类多态调用。
- `ByteRingBuffer` FIFO、空、满、环绕、窥视、清空和最小容量。
- `MockUart` 配置、发送、接收、溢出、生命周期和多态调用。
- `ErrorCode` 枚举比较与字符串转换。

## 编译约束

生产静态库 `robot_hal`：

- MSVC：`/W4 /permissive- /EHs-c- /GR-`
- GCC/Clang：`-Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti`

PC 测试程序需要控制台标准库支持，因此 MSVC 测试目标单独使用 `/EHsc`。CMake 会先清理生成器注入的全局异常选项，避免 `/EHsc` 与 `/EHs-c-` 产生 `D9025` 冲突警告。

## 当前进度

| 天数 | 内容 | 状态 |
|---|---|---|
| Day1 | 工程骨架与 `Peripheral` 生命周期 | 已完成 |
| Day2 | `Gpio` 抽象接口与 `MockGpio` | 已完成 |
| Day3 | 固定容量字节环形缓冲区 | 已完成 |
| Day4 | `Uart` 抽象接口与 `MockUart` | 已完成 |
| Day5 | 通用错误码和基础单位类型 | 已完成 |
| Day6 | 资源所有权与严格编译约束 | 已完成 |
| Day7 | 跨层依赖、编译期资源语义和接口替换检查 | 功能完成，待规范整改 |
| Day8 | 第一阶段评审与文档整理 | 待完成 |

下一步是完成 Day8 阶段评审文档，然后进入 Day9 PID 控制器的 PC 端实现。

## 文档

- [嵌入式 C++ 编码规范](CODING_STANDARD.md)

