/**
 * @file voltage_limit.hpp
 * @brief 电压矢量限幅模块，αβ静止坐标系电压限幅工具集
 * @details
 * 提供两种限幅算法：六边形I区限幅、圆形内切圆限幅；
 * 纯数学算法，无硬件依赖，可PC单元测试，无运行时状态，函数可重入。
 * @note
 *  1. hexagon_limit：六边形I区限幅，允许更高输出电压，沿矢量原始方向径向整体缩放；
 *  2. circle_limit：圆形内切圆限幅，径向等比例缩放，**保持矢量角度不变**
 *  3. SVPWM模块不再内置限幅；使用SVPWM前，必须调用本模块完成电压限幅。
 *  4. 本类为最终实现类，禁止继承
 */
#ifndef ROBOT_FOC_ALGORITHM_VOLTAGE_LIMIT_HPP
#define ROBOT_FOC_ALGORITHM_VOLTAGE_LIMIT_HPP

#include "units.hpp"
#include "error_code.hpp"
#include "control_type.hpp"

namespace robot_foc::algorithm
{
    struct VoltageLimitOut
    {
        common::AlphaBetaVoltage alphabeta_out_ {};
        bool is_limited_ {false};
    };

    class VoltageLimit final
    {
    public:
        // 禁止实例化，全部使用静态接口调用
        VoltageLimit() = delete;
        ~VoltageLimit() = delete;
        VoltageLimit(const VoltageLimit&) = delete;
        VoltageLimit& operator=(const VoltageLimit&) = delete;
        VoltageLimit(VoltageLimit&&) = delete;
        VoltageLimit& operator=(VoltageLimit&&) = delete;
        /**
         * @brief αβ电压矢量【六边形I区限幅】
         * @param[in]  alphabeta_in  原始未限幅αβ参考电压矢量
         * @param[in]  v_bus         直流母线电压
         * @param[out] result_out    输出限幅结果结构体（含矢量+is_limited标志）
         * @retval common::ErrorCode::Ok 限幅完成，矢量在线性区或已钳位
         * @retval common::ErrorCode::ErrorInvalidParam 输入非法（NaN/Inf、Vbus<=0）
         * @note
         *  矢量超出六边形边界时沿原方向径向缩放，保持矢量角度不变；
         *  最大输出随角度变化，α轴方向最大可达 (2.0F/3.0F)*Vbus。
         *  输入非法时不修改 result_out。
         */
        [[nodiscard]] static common::ErrorCode hexagon_limit(
            const common::AlphaBetaVoltage& alphabeta_in,
            const common::Volts v_bus,
            VoltageLimitOut& result_out
        ) noexcept;
        /**
         * @brief αβ电压矢量【圆形内切圆限幅】
         * @param[in]  alphabeta_in  原始未限幅αβ参考电压矢量
         * @param[in]  v_bus         直流母线电压
         * @param[out] result_out    输出限幅结果结构体（含矢量+is_limited标志）
         * @retval common::ErrorCode::Ok 限幅完成，矢量在线性区或已钳位
         * @retval common::ErrorCode::ErrorInvalidParam 输入非法（NaN/Inf、Vbus<=0）
         * @note
         *  径向等比例缩放，**严格保持矢量角度不变**；
         *  最大幅值固定为 Vbus / sqrt(3)。
         *  输入非法或计算结果不可表示时不修改 result_out。
         */
        [[nodiscard]] static common::ErrorCode circle_limit(
            const common::AlphaBetaVoltage& alphabeta_in,
            const common::Volts v_bus,
            VoltageLimitOut& result_out
        ) noexcept;
    };
} // namespace robot_foc::algorithm
#endif // ROBOT_FOC_ALGORITHM_VOLTAGE_LIMIT_HPP
