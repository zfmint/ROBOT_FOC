/**
 * @file svpwm.hpp
 * @brief 七段式SVPWM空间矢量调制，等幅值体系
 * @details
 * 输入静止轴αβ电压矢量，输出三相PWM占空比；
 * 无运行时历史状态，可重入。
 * @note
 *  1. 输出duty范围 0.0F ~ 1.0F
 *  2. 七段对称调制，零矢量居中
 */
#ifndef ROBOT_FOC_ALGORITHM_SVPWM_HPP
#define ROBOT_FOC_ALGORITHM_SVPWM_HPP

#include <cstdint>

#include "units.hpp"
#include "error_code.hpp"
#include "foc_transform.hpp"

namespace robot_foc::algorithm
{
    /**
     * @struct SvpwmDuty
     * @brief SVPWM输出三相占空比数据包
     * @param a_duty_ A相归一化占空比 [0.0F, 1.0F]
     * @param b_duty_ B相归一化占空比 [0.0F, 1.0F]
     * @param c_duty_ C相归一化占空比 [0.0F, 1.0F]
     */
    struct SvpwmDuty
    {
        float a_duty_;
        float b_duty_;
        float c_duty_;
    };

    class Svpwm
    {
    private:
        /**
         * @brief 扇区折叠：将任意扇区αβ矢量映射到第一扇区
         * @param[in]  alphabeta_in   原始αβ电压矢量
         * @param[out] alphabeta_out  映射到第一扇区后的αβ矢量
         * @param[out] sector_out     返回原始矢量所在扇区号（1~6），用于后续还原
         * @retval common::ErrorCode::Ok 映射成功
         * @retval common::ErrorCode::ErrorInvalidParam 输入非法
         * @warning 必须配合 svpwm_back_sector 使用，否则输出占空比相位错误
         */
        [[nodiscard]] static common::ErrorCode svpwm_to_one_sector(
            const AlphaBetaVoltage& alphabeta_in,
            AlphaBetaVoltage& alphabeta_out,
            std::uint8_t& sector_out
        ) noexcept;


        /**
         * @brief 扇区还原：第一扇区计算得到的占空比，根据原始扇区换回真实相序
         * @param[in]  sector_in      原始扇区号，由svpwm_to_one_sector输出得到
         * @param[in]  duty_in        第一扇区算出的临时占空比
         * @param[out] duty_out       还原到原始扇区的最终三相占空比
         * @retval common::ErrorCode::Ok 还原成功
         * @retval common::ErrorCode::ErrorInvalidParam 扇区号不在1~6范围内
         */
        [[nodiscard]] static common::ErrorCode svpwm_back_sector(
            const std::uint8_t sector_in,
            const SvpwmDuty& duty_in,
            SvpwmDuty& duty_out
        ) noexcept;
    
    public:
        Svpwm() noexcept = default;
        ~Svpwm() noexcept = default;

        /**
         * @brief αβ电压矢量限幅
         * @param[in]  alphabeta_in  原始αβ参考电压矢量
         * @param[in]  v_bus  直流母线电压
         * @param[out] alphabeta_out 限幅后αβ电压矢量
         * @retval common::ErrorCode::Ok 限幅完成，矢量在线性区或已钳位
         * @retval common::ErrorCode::ErrorInvalidParam 输入非法
         * @note 仅做矢量缩放，不改变矢量角度
         */
        [[nodiscard]] common::ErrorCode voltagelimit(
            const AlphaBetaVoltage& alphabeta_in,
            const common::Volts v_bus,
            AlphaBetaVoltage& alphabeta_out
        ) const noexcept;

        /**
         * @brief SVPWM主计算：αβ电压矢量 + 母线电压 → 三相占空比
         * @param[in]  alphabeta_in 静止坐标系αβ参考电压矢量（限幅后）
         * @param[in]  v_bus        直流母线电压Vbus
         * @param[out] duty_out     输出三相归一化占空比
         * @retval common::ErrorCode::Ok 计算成功
         * @retval common::ErrorCode::ErrorInvalidParam Vbus非法、输入矢量异常
         * @note
         *  - 输入：已经完成限幅的电压矢量
         *  - 调制方式：七段式对称SVPWM，零矢量居中
         *  - 输入电压指令要求先完成限幅
         *  - 输出占空比需要结合PWM重装值换算为比较寄存器值
         *  - 不含死区补偿、最小脉宽处理
         */
            [[nodiscard]] common::ErrorCode svpwm_calculate(
            const AlphaBetaVoltage& alphabeta_in,
            common::Volts v_bus,
            SvpwmDuty& duty_out
        ) const noexcept;

    };    
} // namespace robot_foc::algorithm

#endif
