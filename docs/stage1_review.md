# Robot_FOC 第一阶段评审记录

## 一、评审信息

- 阶段：Day1～Day8，嵌入式 C++ 架构筑基
- 评审日期：2026-10-10
- 评审范围：CMake 工程、HAL 抽象、PC Mock 驱动、固定容量缓冲区、UART 接口、错误码、编译约束和阶段文档
- Python、STM32F4、FreeRTOS 和算法层不属于本阶段已实现范围

## 二、阶段门禁结果

| 检查项 | 结果 | 证据或说明 |
|---|---|---|
| `robot_hal_core` 可构建 | 通过 | CMake 生产核心静态库，关闭异常和 RTTI |
| `robot_hal_mock` 可构建 | 通过 | PC 构建时生成，交叉编译时跳过 |
| `pc_unit_test` 可构建 | 通过 | 链接 core 与 Mock 静态库 |
| MSVC 严格警告 | 通过 | `/W4 /permissive-`，最终构建无新增警告 |
| 生产库异常/RTTI约束 | 通过 | `/EHs-c- /GR-`；GCC/Clang 使用 `-fno-exceptions -fno-rtti` |
| PC 单元测试 | 通过 | 全部测试通过，失败数为 0 |
| 测试失败退出码 | 通过 | `pc_test_main.cpp` 在失败数非零时返回 `EXIT_FAILURE` |
| 测试与生产头文件隔离 | 通过 | `Test/Inc` 仅通过 `PRIVATE` 暴露给 `pc_unit_test` |
| Python 自动化测试 | 暂缓 | 按项目规则，待 PC C++ 接口稳定后接入 |

阶段门禁结论：PC 基础组件能够构建，测试程序具备失败非零退出行为，Day8 阶段门禁通过。

## 三、当前类关系

```text
Peripheral
├── Gpio
│   └── MockGpio
└── Uart<RxBufferCapacity>
    └── MockUart<RxBufferCapacity, TxBufferCapacity>

Uart / MockUart
└── ByteRingBuffer<Capacity>（组合）
```

算法层和 RTOS 调度层目前只有规划目录，没有可验收的实现类。

## 四、阶段一技术决策记录

### D-001：PC C++ 优先，Python 暂缓

- 决策：先用 PC C++ 完成接口、生命周期和边界测试，再接入 Python/pytest。
- 原因：避免 Python 绑定层掩盖 C++ 接口和资源语义问题。
- 影响：当前构建不依赖 Python、pybind11 或 pytest。

### D-002：生产核心库与 Mock 驱动分离

- 决策：生产代码使用 `robot_hal_core`，PC Mock 使用 `robot_hal_mock`。
- 原因：STM32 固件不能链接 PC 仿真实现，避免 Mock 依赖进入固件。
- 影响：`pc_unit_test` 同时链接两个库；交叉编译时仅构建核心库。

### D-003：生产代码关闭异常和 RTTI

- 决策：生产目标关闭异常和 RTTI，错误通过 `ErrorCode`、bool 或明确结果返回。
- 原因：嵌入式固件资源受限，且项目要求禁止异常和 RTTI。
- 影响：PC 测试目标可以使用独立的 `/EHsc` 控制台配置，但不能将其配置传递给生产库。

### D-004：外设对象禁止复制和移动

- 决策：`Peripheral`、`Gpio`、`Uart`、Mock 驱动和固定容量缓冲区显式删除复制/移动操作。
- 原因：防止硬件句柄、生命周期状态和资源所有权被隐式复制或转移。
- 影响：使用引用、基类引用或依赖注入传递设备对象；关键类型使用 `static_assert` 编译期校验。

### D-005：ByteRingBuffer 不承担并发同步

- 决策：环形缓冲区只提供固定容量 FIFO，不内置原子操作、锁或 FreeRTOS 依赖。
- 原因：保持 HAL 工具可移植、可在 PC 独立测试。
- 影响：STM32 阶段必须由 ISR/任务边界、临界区、队列或专用 SPSC 结构负责同步。

### D-006：HAL 与具体驱动解耦

- 决策：算法和上层业务只依赖 HAL 抽象，不直接依赖 `MockGpio` 或未来的 STM32 驱动类型。
- 原因：替换硬件实现时，上层调用接口不应改变。
- 影响：当前 GPIO 通用测试通过 `Gpio&` 接收对象，后续可替换为真实 GPIO 实现。

## 五、阶段问题清单

| 编号 | 问题 | 当前状态 | 后续计划 |
|---|---|---|---|
| I-001 | `ALGORITHM`、`RTOS` 目录暂无实现 | 已记录 | Day9 起实现 PID、滤波、FOC 和业务状态机 |
| I-002 | STM32F4 交叉编译工具链文件尚未纳入仓库 | 已记录 | Day19 建立交叉编译工程 |
| I-003 | Python/pytest/pybind11 尚未接入 | 按计划暂缓 | PC C++ 接口稳定后再接入 |
| I-004 | UART 缓冲区当前不提供 ISR/任务并发保护 | 已明确约束 | Day21 设计中断接收和任务解析边界 |
| I-005 | FreeRTOS 任务、队列、信号量尚未实现 | 已记录 | Day27～Day34 建立调度层 |
| I-006 | `DEVELOPMENT_PLAN.md` 当前未纳入 Git 跟踪 | 待处理 | 将正式项目计划纳入文档版本管理 |

以上问题不阻塞 Day8 的 PC 架构阶段门禁，但在进入 STM32 或 FreeRTOS 阶段前必须重新评审。

## 六、下一阶段入口条件

进入 Day9 前应保持：

1. `robot_hal_core`、`robot_hal_mock` 和 `pc_unit_test` 的目标命名及依赖关系不被破坏。
2. 新增算法类继续使用固定容量、无异常、无 RTTI 的受控 C++ 子集。
3. 每个算法模块先增加 PC C++ 边界测试，再进入后续硬件移植。
4. 不把 Python 测试结果作为 C++ 编译和接口契约测试的替代品。
