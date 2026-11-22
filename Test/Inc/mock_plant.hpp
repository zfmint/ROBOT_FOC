/**
 * @file mock_plant.hpp
 * @brief PC单元测试：被控对象仿真模型集合
 * @details
 * 全部模型仅用于PC单元测试，**禁止编译进MCU固件**
 * 迭代路线：
 *  1. MockFirstOrderPlant：SISO单通道一阶惯性环节
 *  2. AlphaBetaFirstOrderPlant：α‑β双通道独立一阶惯性(Stage2，本次新增)，无反电动势、无电气耦合
 *  后续扩展：MockSpmsmRl SPMSM RL电机、带噪声模型等
 */

#ifndef ROBOT_FOC_TEST_MOCK_PLANT_HPP
#define ROBOT_FOC_TEST_MOCK_PLANT_HPP

#include "error_code.hpp"
#include "control_type.hpp"

namespace robot_foc::test
{
    /**
     * @brief SISO 单输入单输出一阶惯性离散模型
     * @param ts 仿真步长 s
     * @param tau 时间常数 s
     * @note 数学：y_{k+1}=y_k + ts*(u‑y_k)/tau
     */
    class MockFirstOrderPlant
    {
    private:
        float y_{0.0F}; ///< 对象当前输出状态
        float ts_{0.0F}; ///< 采样周期，单位：秒
        float tau_{0.0F}; ///< 惯性时间常数，单位：秒，必须大于0

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

    /**
     * @brief α‑β坐标系双通道一阶惯性电机模型
     * @details α、β通道互相独立，无反电动势，无旋转电气耦合；
     * 适合电流内环单元测试，验证电流跟踪、抗饱和、SVPWM全链路。
     * 仅适合电角度固定场景；转子持续旋转仿真请使用后续MockSpmsmRl。
     * @param ts 仿真步长，必须与控制器sample_time保持一致
     * @param tau 时间常数，αβ通道共用同一时间常数
     */
    class AlphaBetaFirstOrderPlant
    {
    private:
        MockFirstOrderPlant plant_alpha_; ///< α轴
        MockFirstOrderPlant plant_beta_; ///< β轴
        common::AlphaBetaVoltage AlphaBetaVolts {}; ///< 当前输出状态
    public:
        /**
         * @brief 构造函数，指定采样周期与惯性时间常数
         * @param[in] ts 采样周期，单位：秒，必须 > 0
         * @param[in] tau 惯性时间常数，单位：秒，必须 > 0
         * @note 非法参数会终止测试程序，避免产生除零或无效仿真结果
         */
        explicit AlphaBetaFirstOrderPlant(float ts, float tau) noexcept;

        ~AlphaBetaFirstOrderPlant() noexcept = default;
        
        /**
         * @brief 禁止拷贝、禁止移动
         * @note 对象携带仿真状态，防止无意拷贝造成测试隐蔽bug
         */
        AlphaBetaFirstOrderPlant(const AlphaBetaFirstOrderPlant&) = delete;
        AlphaBetaFirstOrderPlant& operator=(const AlphaBetaFirstOrderPlant&) = delete;
        AlphaBetaFirstOrderPlant(AlphaBetaFirstOrderPlant&&) = delete;
        AlphaBetaFirstOrderPlant& operator=(AlphaBetaFirstOrderPlant&&) = delete;

        /**
         * @brief 执行一步迭代，输入控制量volts_input，返回新的对象输出（反馈）
         * @param[in] volts_input PID输出的控制量
         * @return 更新后的被控对象输出
         */
        [[nodiscard]] common::ErrorCode update(common::AlphaBetaVoltage volts_input) noexcept;

        /**
         * @brief 重置对象内部状态，输出y清零
         */
        void reset() noexcept;

    };
} // namespace robot_foc::test

#endif // ROBOT_FOC_TEST_MOCK_PLANT_HPP
