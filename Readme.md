# Robot_FOC

基于 STM32F4 与 FreeRTOS 的机器人移动底盘嵌入式控制系统。项目采用“PC 端 C++ 先实现和验证，再移植到 STM32”的开发流程，目标覆盖 FOC 电机控制、差速底盘运动学、车载档位交互、故障保护、外设驱动封装和实时多任务调度。

当前处于第二阶段“核心算法和 PC 验证”，Day1～Day14 已完成，Day15 速度环正在整理需求。2026-11-23 核查：MSVC Debug 构建通过，PC 测试共执行 1214 次断言，全部通过。Python 自动化测试继续暂缓，STM32 移植和 FreeRTOS 集成尚未开展。

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
- 位置式 PID、增量式 PID 和电流环用 PI 控制器，支持积分管理与复位。
- 固定容量滑动平均滤波、编码器测速。
- Clarke / Park 变换及逆变换、电压矢量限幅和 SVPWM 占空比计算。
- `CurrentLoop` 串接 Id/Iq 双 PI、坐标变换、电压限幅和 SVPWM；全部计算成功后提交积分状态与占空比。
- PC 测试模型支持一阶对象、编码器模拟和 αβ 电流对象，用于算法闭环验证。

## 目录结构

```text
Robot_FOC/
├── Common/                 # 错误码、物理量单位和控制类型
├── Inc/
│   ├── ALGORITHM/          # PID、滤波、测速、FOC 和电流环接口
│   ├── DRIVERS/             # PC Mock 设备驱动
│   └── HAL/                 # 外设抽象接口和固定容量工具
├── Src/
│   ├── ALGORITHM/          # 算法实现
│   ├── DRIVERS/             # Mock 驱动实现
│   └── HAL/                 # HAL 非模板实现
├── Test/
│   ├── Inc/                 # PC 测试辅助接口
│   ├── Src/                 # C++ 单元测试
│   ├── mock/               # 测试模型（Inc/ 与 Src/）
│   ├── pc_test_main.cpp     # PC 测试入口
│   └── pybind/              # 预留目录，当前未接入构建
├── docs/                   # 架构、构建、阶段评审和设计笔记
├── CMakeLists.txt
├── DEVELOPMENT_PLAN.md
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

测试程序会执行全部用例并汇总断言总数、成功数和失败数：全部通过时返回 `0`，任何断言失败时返回非零退出码。默认仅显示失败详情与汇总；在 `Test/Inc/test_helper.hpp` 中将 `test_verbose` 设为 `true` 可显示每次成功断言。汇总数量包含循环内重复执行的断言，不代表独立测试场景数。

当前测试覆盖：

- `Peripheral` 初始化、反初始化、重复调用和实例状态隔离。
- `MockGpio` 配置、模式、电平、生命周期和基类多态调用。
- `ByteRingBuffer` FIFO、空、满、环绕、窥视、清空和最小容量。
- `MockUart` 配置、发送、接收、溢出、生命周期和多态调用。
- `ErrorCode` 枚举比较与字符串转换。
- PID / PI 控制器、滑动平均滤波和编码器测速的单元测试与相关闭环仿真。
- FOC 坐标变换、电压限幅和 SVPWM 的数值与边界测试。
- 电流环配置、非法输入、复位、饱和工况占空比范围，以及完整链路的 d/q 电流跟踪。

电流环闭环测试通过三相占空比还原 αβ 电压，驱动 `AlphaBetaFirstOrderPlant`，再经 Park 变换得到 Id/Iq 反馈。当前跟踪场景采用固定电角度 0、24 V 母线、0.1 ms 步长，运行 2500 步，目标电流为 Id=1 A、Iq=2.5 A，最终误差容差为 0.3 A。该结果用于 Day14 的 PC 简化模型验收；动态电角度、真实电机耦合和板端效果仍需后续验证，饱和测试目前只验证运行成功与占空比范围。

## 编译约束

生产核心静态库 `robot_hal_core`，PC Mock 静态库 `robot_hal_mock`：

- MSVC：`/W4 /permissive- /EHs-c- /GR-`
- GCC/Clang：`-Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti`

PC 测试程序需要控制台标准库支持，因此 MSVC 测试目标单独使用 `/EHsc`。CMake 会先清理生成器注入的全局异常选项，避免 `/EHsc` 与 `/EHs-c-` 产生 `D9025` 冲突警告。

## 当前进度

最近核查：2026-11-23；代码基线：`eaef489`，当前分支：`feat/Day15`。以下“已完成”指相应阶段的 PC 实现与验收，不代表板端验证完成。

| 天数 | 内容 | 状态 |
|---|---|---|
| Day1 | 工程骨架与 `Peripheral` 生命周期 | 已完成 |
| Day2 | `Gpio` 抽象接口与 `MockGpio` | 已完成 |
| Day3 | 固定容量字节环形缓冲区 | 已完成 |
| Day4 | `Uart` 抽象接口与 `MockUart` | 已完成 |
| Day5 | 通用错误码和基础单位类型 | 已完成 |
| Day6 | 资源所有权与严格编译约束 | 已完成 |
| Day7 | 跨层依赖、编译期资源语义和接口替换检查 | 已完成 |
| Day8 | 第一阶段评审与文档整理 | 已完成 |
| Day9 | PID控制器 | 已完成 |
| Day10 | 滤波算法 | 已完成 |
| Day11 | 编码器测速 | 已完成 |
| Day12 | FOC 坐标变换 | 已完成 |
| Day13 | SVPWM 与电压限幅 | 已完成 |
| Day14 | Id/Iq 双电流环及 PC 闭环仿真 | 已完成 |
| Day15 | 速度环 | 需求整理中，尚无速度环实现与测试 |
| Day16～18 | 差速运动学、速度平滑、状态机、故障诊断与算法集成 | 待开展 |
| Day19～26 | STM32F4 外设驱动封装 | 待开展 |
| Day27～34 | FreeRTOS 多任务架构 | 待开展 |
| Day35～40 | 整机联调与终验 | 待开展 |

本次验证：`cmake --build build --config Debug` 和 `build/bin/Debug/pc_unit_test.exe` 均返回 0；1214 次断言成功，0 失败。此前 PI 接口编译阻塞和电流环 q 轴跟踪测试失败已解除。

下一步完成 Day15 速度环需求与接口定义：明确目标/反馈速度单位、速度环输出的电流给定值及限幅、采样周期与电流环的调用关系；随后实现速度控制器并验证加速、减速和负载变化。当前速度环设计笔记仍含电流环需求内容，需要先整理为速度环规格。

## 文档

- [完整开发计划](DEVELOPMENT_PLAN.md)（其中的“当前进度”尚未同步，本页记录本次核查结果）
- [嵌入式 C++ 编码规范](CODING_STANDARD.md)
- [架构说明](docs/architecture.md)
- [CMake 构建说明](docs/build_guide.md)
- [第一阶段评审记录](docs/stage1_review.md)
- [Day12 FOC 坐标变换笔记](docs/Thinking/Day12FOCtranslation.md)
- [Day13 SVPWM 笔记](docs/Thinking/Day13.md)
- [Day14 电流环笔记](docs/Thinking/Day14电流环.md)
