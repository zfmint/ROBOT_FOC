#include "HAL/peripheral.hpp"

namespace robot_foc::hal
{
    ///@brief 通过一个公共函数访问initialized_，防止外界函数直接修改initialized_
    bool Peripheral::is_initialized() const noexcept{
        return initialized_;
    }

    /**
     * @brief 将外设标记初始化状态
     * @return true 表示初始化成功；
     * @return false 表示外设已经处初始化状态，无需操作
     * @note 仅在外设未初始化时才修改状态
     * @note 重复调用不会修改状态，失败不改动成员变量
     */
    bool Peripheral::mark_initialized() noexcept{
        if (initialized_){
            return false;
        }
        initialized_ = true;
        return true;
    }

    /**
     * @brief 将外设标记为未初始化状态
     * @return true 表示反初始化成功；
     * @return false 表示外设本就处于未初始化状态，无需操作
     * @note 仅在外设已初始化时才修改状态，状态未变化不改动成员变量
     */
    bool Peripheral::mark_deinitialized() noexcept{
        if (!initialized_){
            return false;
        }
        initialized_ = false;
        return true;
    }
} // namespace robot_foc::hal