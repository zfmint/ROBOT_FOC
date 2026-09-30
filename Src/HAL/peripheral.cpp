#include "HAL/peripheral.hpp"

namespace robot_foc::hal
{
    //通过一个公共函数访问m_inited，防止外界函数直接修改m_inited
    bool Peripheral::is_inited() const
    {
        return m_inited;
    }
} // namespace robot_foc::hal