/**
 * @file svpwm.hpp
 * @brief 七段式SVPWM空间矢量调制，等幅值体系
 * @details
 * 输入静止轴αβ电压矢量，输出三相PWM占空比；
 * 无运行时历史状态，可重入。
 * @note
 *  1. 输出duty范围 0.0F ~ 1.0F
 *  2. 七段对称调制，零矢量居中
 * @verbatim
 * 电压矢量状态表：
 * 	    A	B	C
 * V0	0	0	0
 * V1	1	0	0
 * V2	1	1	0
 * V3	0	1	0
 * V4	0	1	1
 * V5	0	0	1
 * V6	1	0	1
 * V7	1	1	1
 * @endverbatim
 */
#ifndef ROBOT_FOC_ALGORITHM_SVPWM_HPP
#define ROBOT_FOC_ALGORITHM_SVPWM_HPP

#include <cstdint>

#include "units.hpp"
#include "error_code.hpp"
#include "control_type.hpp"

namespace robot_foc::algorithm
{
    /**
     * @struct SvpwmDuty
     * @brief SVPWM输出三相占空比数据包
     * @var a_duty_ A相归一化占空比 [0.0F, 1.0F]
     * @var b_duty_ B相归一化占空比 [0.0F, 1.0F]
     * @var c_duty_ C相归一化占空比 [0.0F, 1.0F]
     */
    struct SvpwmDuty
    {
        float a_duty_;
        float b_duty_;
        float c_duty_;
    };

    /**
     * @struct SvpwmTime
     * @brief SVPWM有效矢量及零矢量作用时间
     * @var t1_ratio_ 第一个有效电压矢量作用时间
     * @var t2_ratio_ 第二个有效电压矢量作用时间
     * @var t0_ratio_ 零电压矢量总作用时间
     * @note [out] 归一化时间比例
     */
    struct SvpwmTime
    {
        float t1_ratio_;
        float t2_ratio_;
        float t0_ratio_;
    };

    class Svpwm
    {
    private:
        /**
         * @brief 判断输入电压矢量所在扇区
         * @param[in]  alphabeta_in   原始αβ电压矢量
         * @param[out] sector_out     返回原始矢量所在扇区号（1~6），用于后续还原
         * @retval common::ErrorCode::Ok 计算成功
         * @retval common::ErrorCode::ErrorInvalidParam 输入非法
         * @warning 必须配合 svpwm_back_sector 使用，否则输出占空比相位错误
         */
        [[nodiscard]] static common::ErrorCode svpwm_detect_sector(
            const common::AlphaBetaVoltage& alphabeta_in,
            std::uint8_t& sector_out
        ) noexcept;
    
        /**
         * @brief 扇区折叠：将任意扇区αβ矢量映射到第一扇区
         * @param[in]  alphabeta_in   原始αβ矢量
         * @param[in]  sector_in     原始矢量所在扇区号（1~6），用于后续还原
         * @param[out] alphabeta_out  映射到第一扇区后的αβ矢量
         * @retval common::ErrorCode::Ok 映射成功
         * @retval common::ErrorCode::ErrorInvalidParam 输入非法
         */
        [[nodiscard]] static common::ErrorCode svpwm_to_one_sector(
            const common::AlphaBetaVoltage& alphabeta_in,
            const std::uint8_t sector_in,
            common::AlphaBetaVoltage& alphabeta_out
        ) noexcept;


        /**
         * @brief 计算电压矢量对应的输出时间
         * @param[in]  alphabeta_in   映射到第一扇区后的αβ矢量
         * @param[in]  v_bus          母线电压
         * @param[out] time_out       返回计算出的T1,T2,T0
         * @retval common::ErrorCode::Ok 还原成功
         * @retval common::ErrorCode::ErrorInvalidParam 输入不在范围内
         */
        [[nodiscard]] static common::ErrorCode svpwm_calculate_time(
            const common::AlphaBetaVoltage& alphabeta_in,
            const common::Volts v_bus,
            SvpwmTime& time_out
        ) noexcept;

        /**
         * @brief 计算电压输出矢量对应的占空比
         * @param[in]  sector_in      原始扇区号，由svpwm_to_one_sector输出得到
         * @param[in]  time_in       计算出的T1,T2,T0
         * @param[out] duty_out       还原到原始扇区的最终三相占空比
         * @retval common::ErrorCode::Ok 还原成功
         * @retval common::ErrorCode::ErrorInvalidParam 输入不在范围内
         */
        [[nodiscard]] static common::ErrorCode svpwm_calculate_duty(
            const std::uint8_t sector_in,
            const SvpwmTime& time_in,
            SvpwmDuty& duty_out
        ) noexcept;

    public:
        //禁止实例化
        Svpwm() = delete;
        ~Svpwm() = delete;
        Svpwm(const Svpwm&) = delete;
        Svpwm& operator=(const Svpwm&) = delete;
        Svpwm(Svpwm&&) = delete;
        Svpwm& operator=(Svpwm&&) = delete;

        /**
         * @brief αβ电压矢量六边形限幅
         * @param[in]  alphabeta_in  原始αβ参考电压矢量
         * @param[in]  v_bus  直流母线电压
         * @param[out] alphabeta_out 限幅后αβ电压矢量
         * @retval common::ErrorCode::Ok 限幅完成，矢量在线性区或已钳位
         * @retval common::ErrorCode::ErrorInvalidParam 输入非法
         * @note 六边形I区限幅；矢量超出六边形边界会缩放，**可能改变矢量角度**。
         * 若需要保持矢量角度不变，请使用圆形内切圆限幅。
         */
        [[nodiscard]] static common::ErrorCode voltage_limit(
            const common::AlphaBetaVoltage& alphabeta_in,
            const common::Volts v_bus,
            common::AlphaBetaVoltage& alphabeta_out
        ) noexcept;

        /**
         * @brief SVPWM主计算：αβ电压矢量 + 母线电压 → 三相占空比
         * @param[in]  alphabeta_in 静止坐标系αβ参考电压矢量（限幅后）
         * @param[in]  v_bus        直流母线电压Vbus
         * @param[out] duty_out     输出三相归一化占空比
         * @retval common::ErrorCode::Ok 计算成功
         * @retval common::ErrorCode::ErrorInvalidParam Vbus非法、输入矢量异常
         * @note
         *  - 输入：电压矢量,内部会完成限幅
         *  - 调制方式：七段式对称SVPWM，零矢量居中
         *  - 输入电压指令要求先完成限幅
         *  - 输出占空比需要结合PWM重装值换算为比较寄存器值
         *  - 不含死区补偿、最小脉宽处理
         */
            [[nodiscard]] static common::ErrorCode svpwm_calculate(
            const common::AlphaBetaVoltage& alphabeta_in,
            common::Volts v_bus,
            SvpwmDuty& duty_out
        ) noexcept;
    };    
} // namespace robot_foc::algorithm

#endif
