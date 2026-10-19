/**
 * @file mock_plant.cpp
 * @brief 单元测试专用：一阶惯性被控对象模型实现
 * @details
 *  仅用于PC端单元测试，**不编译进MCU产品固件**。
 *  实现一阶惯性离散被控对象，用于PID闭环仿真，
 *  验证阶跃响应、积分饱和、抗积分饱和行为。
 * @author Robot_FOC Project
 * @note 仅单元测试使用，业务代码禁止引用本文件
 */

#include <cassert>

#include "mock_plant.hpp"

namespace robot_foc::test
{
    MockFirstOrderPlant::MockFirstOrderPlant(float ts,float tau) noexcept
        : y_(0.0F), ts_(ts), tau_(tau)
    {
        assert(ts > 0.0F);
        assert(tau > 0.0F);
    }

    void MockFirstOrderPlant::reset() noexcept
    {
        y_ = 0.0F;
    }

    [[nodiscard]] float MockFirstOrderPlant::update(float u) noexcept
    {
        y_ = y_ + ts_ * ((u - y_) / tau_);
        return y_;
    }
} // namespace robot_foc::test
