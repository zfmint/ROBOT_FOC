# ROBOT_FOC 架构说明文档

## 一、文档信息

##### 适用范围：全项目
##### 作者：zfmint
##### 最后更新日期 2026.10.9
##### 匹配文档 --决策记录 --构建说明

## 二、系统概述

1. 系统说明
 - 本项目构建目标：基于FREERTOS的机器人移动底盘控制系统
 - 硬件选择：STM32F407ZGT6
2. 核心问题：
 - 硬件平台：能满足市面多数控制器适配
 - 可测试性：每个模块独立进行单元测试
 - 代码复用：按照模块封装，适应不同操作系统
3. 关键设计目标
 - 分层解耦：高内聚、低耦合，层与层之间不依赖
 - 统一外设生命周期，避免资源占用
 - PC单元测试：模拟测试，验证代码逻辑

## 三、总体架构与分层

依赖方向自底向上，箭头指向被依赖方
``` text
DRIVERS -> HAL -> COMMON
ALGORITHM -> HAL -> COMMON
RTOS
```

1. COMMON（公共基础层）
 - 包含内容\
    `error_code.hpp`：统一错误码 `ErrorCode`（Ok / ErrorNotReady / ErrorAlreadyInit / ErrorInvalidParam / ErrorHardwareFault / ErrorUnsupported ）\
    物理单位别名、通用工具函数（按实际内容补充）
 - 职责边界\
    提供全项共用，无硬件相关的基础类型与组件\
    不操作寄存器，不包含外设逻辑，不依赖FREERTOS,不包含业务、代码
 - 依赖方向\
    无依赖
2. HAL（硬件抽象层）
 - 包含内容\
    `Peripheral` 抽象基类：统一外设生命周期契约（`init` / `deinit` / `is_initialized` / 状态标记），禁止拷贝移动，static_assert 编译期校验
    `Gpio` 抽象类：`configure` / `set_high` / `set_low` / `set_mode` / `read_level`
    `Uart` 抽象类：`configure` / `push_rx` / `read_rx` 等
    `ByteRingBuffer`（若按目录归属列于 HAL）：UART 接收缓存
 - 职责边界\
    定义外设统一接口与行为契约；规定生命周期顺序（configure → init → deinit）、错误码返回规则\
    不直接操作 MCU 寄存器；不含硬件底层实现；不依赖 DRIVERS / ALGORITHM / RTOS / FreeRTOS
 - 依赖方向\
    COMMON
3. DRIVERS（硬件驱动层）
 - 包含内容\
    真实硬件驱动（未来阶段）：MCU GPIO、UART 寄存器驱动，实现 HAL 抽象接口（`RealGpio` / `RealUart` 等）\
    Mock 仿真驱动（当前阶段）：`MockGpio` / `MockUart`，仅内存变量模拟，(仅 PC 单元测试编译)
 - 职责边界\
    把 HAL 契约落地为具体硬件行为（或 Mock 的内存模拟行为）\
    不含业务逻辑、不向上暴露寄存器细节、不引入 FOC/算法概念
 - 依赖方向\
    依赖 COMMON + HAL；禁止依赖 ALGORITHM / RTOS
4. ALGORITHM（算法层）
 - 包含内容\
    FOC 控制算法、PID控制算法、电流/速度环、控制状态机、业务任务逻辑
 - 职责边界\
    电机控制算法与业务逻辑，通过 HAL 抽象接口驱动外设\
    不直接操作 DRIVERS 寄存器、不感知底层是真实硬件还是 Mock
 - 依赖方向\
    依赖 COMMON + HAL （+ RTOS）；禁止依赖 DRIVERS
5. RTOS（系统层）
 - 包含内容\
    FreeRTOS 移植/源码，或任务、队列、信号量封装类
 - 职责边界\
    提供任务调度、同步、通信等系统服务\
    不含外设逻辑、不含算法业务
 - 依赖方向\
    无依赖

## 四、核心设计机制

1. 外设生命周期统一
 - 机制说明
 所有HAL外设遵守统一生命周期：`configure -> init -> deinit`\
 由`Peripheral`基类内置状态标记统一维护，派生类只能通过受保护的状态修改参数变更状态，禁止直接读写状态位
 - 设计理念
 初始化顺序可控制、可重复：禁止在init前操作硬件，避免未初始化访问寄存器\
 配置生效后不可更改：init后修改配置可能造成配置不一致，必须deinit后才能重新配置\
 deinit后可以直接复用原配置：不需要重新configure，减少恢复成本\
 可测试：每个阶段具有明确的错误码，通过单元测试断言生命周期时序
 - 保证措施
 `Peripheral` 基类内部维护 `initialized_` 状态位，派生类通过 `mark_initialized()` / `mark_deinitialized()` 修改\
 所有接口返回 `ErrorCode` 而非裸 bool，调用方必须检查\
 单元测试（`run_gpio_common_tests`）逐阶段断言：未 init 拒绝、重复 init 拒绝、init 后 configure 拒绝、deinit 后拒绝、deinit 后直接 init 复用配置。
2. 错误处理机制
 - 机制说明
 外设接口统一使用'ErrorCode'枚举
 - 设计理念
 不使用异常：避免嵌入式环境下Flash/RAM开销\
 编译期间关闭异常：逻辑测试与真实环境保持一致\
 错误频率：外设操作失败概率高，避免反复抛出异常
 - 保证措施
 Cmake关闭异常，开启'[[nodiscard]]'函数标注\
 HAL以及上层接口函数全部返回ErrorCode\
3. 所有权保护机制
 - 机制说明
 外设绑定真实硬件资源，不可复制。不可移动。避免大块内存无意复制
 - 设计理念
 硬件唯一，严格遵守生命周期
 - 保证措施
 显示删除复制、移动函数\
 编译器强制校验

## 五、关键组件\类说明

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
 派生类必须遵守'configure → init → deinit'生命周期\
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
 virtual GpioLevel read_level() const noexcept;
 ```
 - 约束\
 init后才能执行引脚读写\
 init后不能进行configure，必须先deinit\
 禁止拷贝、移动，编译器校验\
 输出模式下调用读电平返回硬件采样值\
 输入模式调用'set_high/set_low'返回'ErrorUnsupported'
3. Uart<RxCap,TxCap>
 - 职责\
 串口外设抽象契约类，定义串口配置、收发字节的统一接口，内置环形缓冲区
 - 关键接口\
 ```text
 [[nodiscard]] virtual ErrorCode configure(const UartConfig& cfg) noexcept;
 [[nodiscard]] virtual ErrorCode push_rx(uint8_t data) noexcept;
 [[nodiscard]] virtual ErrorCode read_rx(uint8_t& out) noexcept;
 ```
 - 约束\
 Tx仅有虚函数，若是模拟PC测试发送，需要在子类中实现\
 必须init成功后才能收发数据\
 init后禁止再次configure，需要先deinit\
 内置 ByteRingBuffer，默认单生产者单消费者；中断与任务并发访问时必须加临界区保护\
 禁止拷贝、移动，编译器校验\
4. ByteRingBuffer<Capacity>
 - 职责\
 编译期定长字节环形缓冲区，用于 UART 等外设数据缓存
 - 关键接口\
 '''text
 [[nodiscard]] ErrorCode push(uint8_t data) noexcept;
 [[nodiscard]] ErrorCode pop(uint8_t& out) noexcept;
 bool is_full() const noexcept;
 bool is_empty() const noexcept;
 '''
 - 约束\
 默认设计为单生产者、单消费者；多生产者 / 多消费者场景必须使用临界区保护，目前仅在注释中约束\
 缓冲区容量在编译期固定，运行时不可修改\
 不做动态内存分配，全部使用栈内存\
 禁止拷贝、移动，编译器校验\

5. MockGpio、MockUart
仅作PC测试使用

## 六、设计约束清单

- [嵌入式 C++ 编码规范](../CODING_STANDARD.md)

## 七、目录结构说明

```text
Robot_FOC/
├── CMakeLists.txt              # 构建脚本：定义 robot_hal 库与 pc_unit_test 测试目标
├── CODING_STANDARD.md          # 编码规范（命名、注释、接口约束）
├── DEVELOPMENT_PLAN.md         # 开发计划（未上传 git）
├── prompt.txt                  # AI 检索提示词工程（未上传 git）
├── README.md                   # 项目说明与快速入门
├── .gitattributes              # Git 换行符 / 文本属性统一配置
├── .gitignore                  # Git 忽略规则（build、编译产物等）
│
├── build/                      # CMake 构建产物目录（不入库）
│
├── docs/                       # 架构、决策记录、阶段验收文档
│   └── architecture.md         # 架构说明文档
│
├── Inc/                        # 头文件目录（全项目声明）
│   ├── ALGORITHM/              # 算法层头文件：FOC / PID 控制算法接口
│   ├── COMMON/                 # 公共基础头文件：ErrorCode 错误码、物理单位别名
│   ├── DRIVERS/                # 硬件驱动头文件：真实驱动与 Mock 仿真驱动声明
│   ├── HAL/                    # 硬件抽象头文件：Peripheral / Gpio / Uart / ByteRingBuffer
│   └── RTOS/                   # 系统层头文件：FreeRTOS 移植 / 封装接口
│
├── Src/                        # 实现文件目录（.cpp 定义）
│   ├── ALGORITHM/              # 算法层实现：PID / FOC（目前空置）
│   ├── DRIVERS/                # 硬件驱动实现：MockGpio / MockUart（当前阶段）
│   ├── HAL/                    # HAL 抽象实现：生命周期、外设状态管理
│   └── RTOS/                   # RTOS 实现：任务 / 队列 / 信号量封装
│
└── Test/                       # PC 单元测试目录（仅 PC_BUILD=ON 编译）
    ├── Inc/                    # 测试辅助头文件（test_helper.hpp、EXPECT 宏）
    ├── Src/                    # 测试用例实现（test_mock_gpio.cpp 等）
    ├── pybind/                 # Python 绑定相关
    └── pc_test_main.cpp        # 测试入口：注册并运行全部测试用例，汇总退出码
```
