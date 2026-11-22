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

#include "mock_plant.hpp"

#include <cstdlib>
namespace robot_foc::test::mock
{
    MockFirstOrderPlant::MockFirstOrderPlant(float ts, float tau) noexcept
        : y_ {0.0F}, ts_ {ts}, tau_ {tau}
    {
        if (!(ts > 0.0F) || !(tau > 0.0F))
        {
            std::abort();
        }
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

    AlphaBetaFirstOrderPlant::AlphaBetaFirstOrderPlant(float ts, float tau) noexcept
        : plant_alpha_ {ts, tau}, plant_beta_ {ts, tau}
    {
    }

    [[nodiscard]] common::AlphaBetaCurrent AlphaBetaFirstOrderPlant::update(
        const common::AlphaBetaVoltage& volts_input
    ) noexcept
    {
        common::AlphaBetaCurrent result {};
        const common::Amperes curr_alpha = plant_alpha_.update(volts_input.alpha_volt_);
        const common::Amperes curr_beta = plant_beta_.update(volts_input.beta_volt_);
        result.alpha_current_ = curr_alpha;
        result.beta_current_ = curr_beta;
        return result;
    }

    void AlphaBetaFirstOrderPlant::reset() noexcept
    {
        plant_alpha_.reset();
        plant_beta_.reset();
    }
} // namespace robot_foc::test::mock
