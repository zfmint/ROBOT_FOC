/**
 * @file peripheral.cpp
 * @brief Peripheral 外设抽象基类实现
 * @details
 *  实现外设生命周期状态管理：初始化/反初始化状态标志的查询与切换；
 *  本文件仅维护内部状态标志，不执行具体硬件操作；
 *  硬件资源申请/释放由各子类（GPIO、UART等）在 init()/deinit() 中实现。
 * @par 生命周期
 *  构造 → configure(可选) → init() → 使用外设 → deinit() → 析构
 * @note 状态标志操作非原子、无并发保护；跨ISR/任务访问需外部临界区保护
 */
#include "HAL/peripheral.hpp"

namespace robot_foc::hal
{

    bool Peripheral::is_initialized() const noexcept
    {
        return initialized_;
    }

    Peripheral::ErrorCode Peripheral::mark_initialized() noexcept
    {
        if (initialized_)
        {
            return Peripheral::ErrorCode::ErrorAlreadyInit;
        }

        initialized_ = true;
        return Peripheral::ErrorCode::Ok;
    }

    Peripheral::ErrorCode Peripheral::mark_deinitialized() noexcept
    {
        if (!initialized_)
        {
            return Peripheral::ErrorCode::ErrorNotReady;
        }

        initialized_ = false;
        return Peripheral::ErrorCode::Ok;
    }

} // namespace robot_foc::hal
