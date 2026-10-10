# ROBOT_FOC 架构说明文档

## 一、文档信息

##### 适用范围：全项目
##### 作者：zfmint
##### 最后更新日期 2026.10.10
##### 相关文档：编码规范、README、CMake 构建脚本

## 二、系统概述

1. 系统说明
 - 本项目构建目标：基于 FreeRTOS 的机器人移动底盘控制系统
 - 硬件选择：STM32F407ZGT6
2. 核心问题：
 - 硬件平台：能满足市面多数控制器适配
 - 可测试性：每个模块独立进行单元测试
 - 代码复用：按照模块封装，适应不同操作系统
3. 关键设计目标
 - 分层解耦：高内聚、低耦合，依赖只能沿规定方向传递
 - 统一外设生命周期，避免资源占用
 - PC单元测试：模拟测试，验证代码逻辑

## 三、总体架构与分层

COMMON 是四层架构之外的公共基础层。依赖箭头指向被依赖方，当前代码只实现了 COMMON、HAL 和部分 DRIVERS；ALGORITHM 与 RTOS 目录为后续阶段预留。

```text
RTOS / 应用集成层（规划）
        ├──> ALGORITHM / 业务层（规划） ──┐
        ├──> DRIVERS / 具体驱动层 ────────┤
        └──> HAL / 抽象接口层 ────────────┘
                    │
                    v
              COMMON / 公共基础层
```

1. COMMON（公共基础层）
 - 包含内容\
    `error_code.hpp`：统一错误码 `ErrorCode`（Ok / ErrorInvalidParam / ErrorNotReady / ErrorAlreadyInit / ErrorBufferFull / ErrorBufferEmpty / ErrorTimeout / ErrorHardwareFault / ErrorBusy / ErrorUnsupported）\
    `units.hpp`：时间、速度、电流、电压和角度等基础单位别名
 - 职责边界\
    提供全项共用，无硬件相关的基础类型与组件\
    不操作寄存器，不包含外设逻辑，不依赖 FreeRTOS，不包含算法和业务代码
 - 依赖方向\
    无依赖
2. HAL（硬件抽象层）
 - 包含内容\
    `Peripheral` 抽象基类：统一外设生命周期契约（`init` / `deinit` / `is_initialized` / 状态标记），禁止拷贝移动，并使用 static_assert 编译期校验
    `Gpio` 抽象类：`configure` / `set_high` / `set_low` / `set_mode` / `read_level`
    `Uart<RxBufferCapacity>` 抽象类：`configure` / `read_rx` / `send_byte` / `send_bytes` 等
    `ByteRingBuffer<Capacity>`：固定容量字节环形缓冲区，供 UART 等设备组合使用
 - 职责边界\
    定义外设统一接口与行为契约；GPIO/UART 设备遵守 configure → init → 使用 → deinit 生命周期\
    不直接操作 MCU 寄存器；不含具体硬件实现；不依赖 DRIVERS / ALGORITHM / RTOS / FreeRTOS
 - 依赖方向\
    COMMON
3. DRIVERS（硬件驱动层）
 - 包含内容\
    真实硬件驱动（未来阶段）：MCU GPIO、UART 寄存器驱动，实现 HAL 抽象接口（`RealGpio` / `RealUart` 等）\
    Mock 仿真驱动（当前阶段）：`MockGpio` / `MockUart`，使用内存变量模拟外设行为
 - 职责边界\
    把 HAL 契约落地为具体硬件行为（或 Mock 的内存模拟行为）\
    不含业务逻辑、不向上暴露寄存器细节、不引入 FOC/算法概念
 - 依赖方向\
    依赖 COMMON + HAL；禁止依赖 ALGORITHM / RTOS。
4. ALGORITHM（算法层）
 - 包含内容\
    FOC 控制算法、PID 控制算法、电流/速度环、运动学、控制状态机和故障诊断
 - 职责边界\
    电机控制算法与业务逻辑通过抽象接口工作，不直接访问具体驱动寄存器；纯算法不依赖 FreeRTOS API\
    不感知底层是真实硬件还是 Mock
 - 依赖方向\
    规划依赖 COMMON + HAL；禁止依赖具体 DRIVERS 和 FreeRTOS API
5. RTOS（系统层）
 - 包含内容\
    FreeRTOS 移植、任务、队列、信号量和任务通知封装。当前目录尚无实现
 - 职责边界\
    提供任务调度、同步、通信等系统服务\
    不含外设逻辑、不含算法业务
 - 依赖方向\
    作为应用集成层，规划依赖 ALGORITHM、HAL 和具体 DRIVERS；不得把调度细节泄漏到纯算法层

## 四、核心设计机制

1. 外设生命周期统一
 - 机制说明
  所有具体可配置外设遵守统一生命周期：`configure -> init -> 使用 -> deinit`；`Peripheral` 本身只定义 init/deinit 契约\
 由`Peripheral`基类内置状态标记统一维护，派生类只能通过受保护的状态修改参数变更状态，禁止直接读写状态位
 - 设计理念
 初始化顺序可控制、可重复：禁止在init前操作硬件，避免未初始化访问寄存器\
 配置生效后不可更改：init后修改配置可能造成配置不一致，必须deinit后才能重新配置\
 deinit后可以直接复用原配置：不需要重新configure，减少恢复成本\
 可测试：每个阶段具有明确的错误码，通过单元测试断言生命周期时序
 - 保证措施
 `Peripheral` 基类内部维护 `initialized_` 状态位，派生类通过 `mark_initialized()` / `mark_deinitialized()` 修改\
  所有可能失败的外设操作返回 `ErrorCode` 并标记 `[[nodiscard]]`；查询类接口按语义返回 bool、计数或引用，调用方不得忽略关键操作结果\
 单元测试（`run_gpio_common_tests`）逐阶段断言：未 init 拒绝、重复 init 拒绝、init 后 configure 拒绝、deinit 后拒绝、deinit 后直接 init 复用配置。
2. 错误处理机制
 - 机制说明
  外设操作统一使用 `ErrorCode` 枚举表达多种失败原因
 - 设计理念
 不使用异常：避免嵌入式环境下Flash/RAM开销\
  生产库编译期间关闭异常；PC 测试目标使用独立的编译选项，不与生产库约束混淆\
 错误频率：外设操作失败概率高，避免反复抛出异常
 - 保证措施
  CMake 对生产库关闭异常和 RTTI，并对生产库、测试目标开启严格警告\
  关键外设操作使用 `[[nodiscard]] ErrorCode`，查询类接口保留明确的 bool、计数或引用类型\
3. 所有权保护机制
 - 机制说明
 外设绑定真实硬件资源，不可复制。不可移动。避免大块内存无意复制
 - 设计理念
 硬件唯一，严格遵守生命周期
 - 保证措施
 显式删除复制、移动函数\
 编译器强制校验

## 五、关键组件与类说明

1. Peripheral
 - 职责\
 所有外设的顶层抽象基类，统一管理外设生命周期与初始化状态
 - 关键接口
 ```text
 [[nodiscard]] virtual ErrorCode init() noexcept;
 [[nodiscard]] virtual ErrorCode deinit() noexcept;
 bool is_initialized() const noexcept;
 ```
 - 约束\
 禁止实例化\
  具体驱动必须遵守'configure → init → 使用 → deinit'生命周期\
 禁止拷贝、移动，编译器校验\
 状态仅能通过基类保护方法修改，不能直接修改'initialized_'标记
2. Gpio
 - 职责\
 GPIO 外设抽象契约类，定义引脚配置、读写电平的统一接口。
 - 关键接口\
 ```text
 [[nodiscard]] virtual ErrorCode configure(const GpioConfig& cfg) noexcept;
 [[nodiscard]] virtual ErrorCode set_high() noexcept;
 [[nodiscard]] virtual ErrorCode set_low() noexcept;
  [[nodiscard]] virtual ErrorCode read_level(GpioLevel& out_level) const noexcept;
 ```
 - 约束\
 init后才能执行引脚读写\
 init后不能进行configure，必须先deinit\
 禁止拷贝、移动，编译器校验\
 输出模式下调用读电平返回硬件采样值\
 输入模式调用'set_high/set_low'返回'ErrorUnsupported'
3. Uart<RxBufferCapacity>
 - 职责\
 串口外设抽象契约类，定义串口配置、收发字节的统一接口，内置环形缓冲区
 - 关键接口\
 ```text
  [[nodiscard]] ErrorCode configure(const UartConfig& cfg) noexcept;
  [[nodiscard]] ErrorCode read_rx(std::uint8_t& out_byte) noexcept;
  [[nodiscard]] virtual ErrorCode send_byte(std::uint8_t byte) noexcept;
  [[nodiscard]] virtual ErrorCode send_bytes(const std::uint8_t* data, std::size_t len) noexcept;
 ```
 - 约束\
  发送接口由子类实现；`push_rx(std::uint8_t)` 为受保护的接收注入接口，供驱动或 Mock 使用\
 必须init成功后才能收发数据\
 init后禁止再次configure，需要先deinit\
  内置 ByteRingBuffer；当前实现不提供原子操作或临界区保护，禁止 ISR 与任务并发访问同一接收缓冲区\
 禁止拷贝、移动，编译器校验\
4. ByteRingBuffer<Capacity>
 - 职责\
 编译期定长字节环形缓冲区，用于 UART 等外设数据缓存
 - 关键接口\
  ```text
  [[nodiscard]] bool push(std::uint8_t byte) noexcept;
  [[nodiscard]] bool pop(std::uint8_t& out_byte) noexcept;
  [[nodiscard]] bool peek(std::uint8_t& out_byte) const noexcept;
  [[nodiscard]] bool full() const noexcept;
  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] constexpr std::size_t capacity() const noexcept;
  void clear() noexcept;
  ```
 - 约束\
  非并发安全；多生产者 / 多消费者以及 ISR 与任务并发场景必须由上层增加同步或临界区保护\
 缓冲区容量在编译期固定，运行时不可修改\
  不做动态内存分配，容量和存储空间在编译期固定\
 禁止拷贝、移动，编译器校验\

5. MockGpio、MockUart
`MockGpio` 与 `MockUart` 用于 PC 行为验证，不代表 STM32 真实寄存器驱动。当前 CMake 已将 MockGpio 放入独立的 `robot_hal_mock` 目标，固件只需链接 `robot_hal_core`。

## 六、设计约束清单

- [嵌入式 C++ 编码规范](../CODING_STANDARD.md)

## 七、目录结构说明

```text
Robot_FOC/
├── CMakeLists.txt              # 构建脚本：定义 robot_hal_core、robot_hal_mock 与 pc_unit_test 目标
├── CODING_STANDARD.md          # 编码规范（命名、注释、接口约束）
├── DEVELOPMENT_PLAN.md         # 开发计划（未上传 git）
├── prompt.txt                  # AI 检索提示词工程（未上传 git）
├── Readme.md                   # 项目说明与快速入门
├── .gitattributes              # Git 换行符 / 文本属性统一配置
├── .gitignore                  # Git 忽略规则（build、编译产物等）
│
├── build/                      # CMake 构建产物目录（不入库）
│
├── docs/                       # 架构和阶段评审文档
│   ├── architecture.md         # 架构说明文档
│   ├── build_guide.md          # CMake、PC 测试和交叉编译说明
│   └── stage1_review.md        # Day1～Day8 阶段评审、问题和决策记录
│
├── Inc/                        # 头文件目录（全项目声明）
│   ├── ALGORITHM/              # 算法层头文件：规划目录，当前尚无实现
│   ├── COMMON/                 # 公共基础头文件：ErrorCode 错误码、物理单位别名
│   ├── DRIVERS/                # 硬件驱动头文件：真实驱动与 Mock 仿真驱动声明
│   ├── HAL/                    # 硬件抽象头文件：Peripheral / Gpio / Uart / ByteRingBuffer
│   └── RTOS/                   # 系统层头文件：规划目录，当前尚无实现
│
├── Src/                        # 实现文件目录（.cpp 定义）
│   ├── ALGORITHM/              # 算法层实现：PID / FOC（目前空置）
│   ├── DRIVERS/                # 硬件驱动实现：MockGpio；MockUart 为头文件模板
│   ├── HAL/                    # HAL 抽象实现：生命周期、外设状态管理
│   └── RTOS/                   # RTOS 实现：规划目录，当前尚无实现
│
└── Test/                       # PC 单元测试目录（非交叉编译时由 CMake 构建）
    ├── Inc/                    # 测试辅助头文件（test_helper.hpp、EXPECT 宏）
    ├── Src/                    # 测试用例实现（test_mock_gpio.cpp 等）
    ├── pybind/                 # Python 绑定相关
    └── pc_test_main.cpp        # 测试入口：注册并运行全部测试用例，汇总退出码
```

## 八、当前构建与验证

当前 CMake 定义两个目标：

- `robot_hal_core`：生产核心静态库，MSVC 使用 `/W4 /permissive- /EHs-c- /GR-`；GCC/Clang 使用严格警告并关闭异常、RTTI。
- `robot_hal_mock`：非交叉编译时生成的 PC Mock 驱动静态库。
- `pc_unit_test`：非交叉编译时生成的 PC 测试程序，链接 `robot_hal_core` 和 `robot_hal_mock`，并使用测试目录中的测试辅助代码。

PC 验证命令：

```powershell
cmake -S . -B build
cmake --build build --config Debug --clean-first
.\build\bin\Debug\pc_unit_test.exe
```

测试失败时返回非零退出码。Python、pybind11 和 pytest 当前尚未接入构建流程，不能将其描述为已完成的自动化测试能力。

## 九、阶段状态与待办

- Day1～Day7：PC 端基础架构、GPIO、环形缓冲区、UART、错误码、编译约束和跨层依赖检查已完成。
- Day8：架构说明、CMake/PC 构建说明、阶段问题清单和技术决策记录已建立。
- Day9 以后：实现 PID、滤波、FOC、运动学、状态机，再进入 STM32 和 FreeRTOS 移植。
- `Inc/ALGORITHM`、`Src/ALGORITHM`、`Inc/RTOS`、`Src/RTOS` 当前仅为规划目录，不应视为已实现功能。
