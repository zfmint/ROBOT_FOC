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
 *  Ts：采样周期【秒】，必须和PID calculate调用周期保持一致
 *  tau：惯性时间常数【秒】，必须 > 0
 * @author Robot_FOC Project
 * @note 仅单元测试使用，不编译进MCU固件
 */

#ifndef ROBOT_FOC_TEST_MOCK_PLANT_HPP
#define ROBOT_FOC_TEST_MOCK_PLANT_HPP

namespace robot_foc::test
{
    /**
     * @brief 一阶惯性被控对象Mock模型
     * @note 仅PC单元测试仿真，不可用于固件运行
     */
    class MockFirstOrderPlant
    {
    private:
        float y_{0.0F};      ///< 对象当前输出状态
        float ts_{0.0F};     ///< 采样周期，单位：秒
        float tau_{0.0F};    ///< 惯性时间常数，单位：秒，必须大于0

    public:
        /**
         * @brief 构造函数，指定采样周期与惯性时间常数
         * @param[in] ts 采样周期，单位：秒，必须 > 0
         * @param[in] tau 惯性时间常数，单位：秒，必须 > 0
         * @note 非法参数会终止测试程序，避免产生除零或无效仿真结果
         */
        explicit MockFirstOrderPlant(float ts, float tau) noexcept;

        ~MockFirstOrderPlant() noexcept = default;

        /**
         * @brief 禁止拷贝、禁止移动
         * @note 对象携带仿真状态，防止无意拷贝造成测试隐蔽bug
         */
        MockFirstOrderPlant(const MockFirstOrderPlant&) = delete;
        MockFirstOrderPlant& operator=(const MockFirstOrderPlant&) = delete;
        MockFirstOrderPlant(MockFirstOrderPlant&&) = delete;
        MockFirstOrderPlant& operator=(MockFirstOrderPlant&&) = delete;

        /**
         * @brief 执行一步迭代，输入控制量u，返回新的对象输出（反馈）
         * @param[in] u PID输出的控制量
         * @return 更新后的被控对象输出
         */
        [[nodiscard]] float update(float u) noexcept;

        /**
         * @brief 重置对象内部状态，输出y清零
         */
        void reset() noexcept;
    };
} // namespace robot_foc::test

#endif // ROBOT_FOC_TEST_MOCK_PLANT_HPP
