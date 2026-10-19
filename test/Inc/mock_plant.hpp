/**
 * @file mock_plant.hpp
 * @brief 单元测试专用：被控对象仿真模型
 * @details
 *  仅用于PC端单元测试，**产品固件禁止包含本头文件**。
 *  实现一阶惯性离散模型，模拟带惯性的被控对象（如电机转速），
 *  用于闭环仿真，验证PID阶跃响应、积分饱和、抗饱和特性。
 *
 *  离散公式： y(k) = y(k-1) + Ts * (u(k) - y(k-1)) / tau
 *  y：对象输出（反馈值，送入PID）
 *  u：PID输出控制输入
 *  Ts：采样周期，必须和PID calculate调用周期保持一致
 * @author Robot_FOC Project
 * @note 仅单元测试使用，不编译进MCU固件
 */

#pragma once

namespace robot_foc::test
{
    /**
     * @brief 一阶惯性被控对象Mock模型
     */
    class MockFirstOrderPlant
    {
    private:
        //对象当前输出状态
        float y_;
        //采样周期
        float ts_;
        //时间常数
        float tau_;
    public:
        /// @brief 构造函数，指定采样周期
        /// @param ts 采样周期 Ts，单位：秒
        explicit MockFirstOrderPlant(float ts,float tau) noexcept;

        /// @brief 执行一步迭代，输入控制量u，返回新的对象输出（反馈）
        /// @param u PID输出的控制量
        /// @return 更新后的被控对象输出
        [[nodiscard]] float update(float u) noexcept;

        /// @brief 重置对象内部状态
        void reset() noexcept;
    };
} // namespace robot_foc::test
