/**
 * @file pid.hpp
 * @brief Pid算法定义。提供三种Pid算法：电流环Pi、位置式Pid、增量式Pid
 * @details
 *  用途： 通用PID控制算法，用于闭环控制
 *  特性：
 *      1. 电流环PI PiController
 *          - 输入：设定值setpoint，反馈值feedback
 *          - 参数：kp，ki
 *          - 功能：完整积分项，支持积分限幅，reset()清零积分与历史误差状态
 *      2. 位置式PID PositionalPid
 *          - 输入：设定值setpoint，反馈值feedback
 *          - 参数：kp，ki，kd
 *          - 功能：完整积分项，支持积分限幅、输出限幅，reset()清零积分与历史误差状态
 *      3. 增量式PID IncrementalPid
 *          - 输入：设定值setpoint，反馈值feedback
 *          - 参数：kp，ki，kd
 *          - 功能：输出控制增量Δu，支持增量限幅，reset()清零连续3次历史误差
 * @note
 *  公共约束：
 *      1. 全部使用float浮点数运算
 *      2. 纯数学算法，无硬件、RTOS依赖，仅依赖COMMON基础类型
 *      3. 两个独立类，互不耦合
 *      4. 调用约定：每次calculate()调用间隔必须为固定采样周期
 *
 * @warning
 *  位置式PID：输出为绝对控制量，一旦计算出错，直接满量程输出，冲击大
 *  增量式PID：输出为控制量的变化量，只保存最近3次误差，计算出错扰动很小，安全性高
 *          执行器必须自带位置保持，长期运行有缓慢漂移
 */

#ifndef ROBOT_FOC_ALGORITHM_PID_HPP
#define ROBOT_FOC_ALGORITHM_PID_HPP

#include "units.hpp"
#include "error_code.hpp"
#include "control_type.hpp"

namespace robot_foc::algorithm
{
    /**
     * @brief PI控制器基础配置
     * @var kp_ 比例系数
     * @var ki_ 积分系数
     * @var sample_time_ 采样时间
     * @var integral_min_ 积分下限
     * @var integral_max_ 积分上限
     */
    struct PiConfig final
    {
        float kp_{0.0F};
        float ki_{0.0F};
        common::Seconds sample_time_{0.0F};
        float integral_min_{0.0F};
        float integral_max_{0.0F};
    };
    
    /**
     * @brief PI控制前计算内容
     * @var error_ 控制误差
     * @var request_output_ 尚未进行共同限幅的原始控制量
     */
    struct PiPreview
    {
        common::Amperes error_{0.0F};
        common::Amperes request_output_{0.0F};
    };
    
    /**
     * @brief PI下周期内容
     * @var integral_out_ 更新后的积分缓存
     */
    struct PiNextState
    {
        float integral_out_{0.0F};
    };
    
    /**
     * @brief PI控制器(适配电流环控制)
     */
    class PiController final
    {
    private:
        // PI控制器基础配置
        PiConfig piconfig_{};
        // 积分运行状态,已经提交的积分输出贡献
        float integral_output_{0.0F};
        // 控制器配置完成标志位
        bool configured_{false};
        // 离散积分系数 Ki * Ts，单位 V/A
        float ki_step_{0.0F};
    public:
        PiController() noexcept = default;
        ~PiController() noexcept = default;

        // 禁止拷贝、禁止移动，防止无意复制控制器状态
        PiController(const PiController&) = delete;
        PiController& operator=(const PiController&) = delete;
        PiController(PiController&&) = delete;
        PiController& operator=(PiController&&) = delete;
        
        /**
         * @brief 预计算本周期PI原始控制输出（未限幅）
         * @details 根据电流设定值与反馈值，计算误差、比例项、原始积分项，
         * @param[in] setpoint DQ轴电流目标设定值
         * @param[in] feedback DQ轴电流采样反馈值
         * @param[out] preview_out PI预计算结果（原始未限幅电压、误差、比例项、原始积分项等）
         * @retval ErrorCode::Ok 预计算正常完成
         * @retval 其他错误码：输入参数非法
         */
        [[nodiscard]] common::ErrorCode preview(
            const common::Amperes setpoint,
            const common::Amperes feedback,
            PiPreview& preview_out
        ) const noexcept;

        /**
         * @brief 限幅后，准备PI下一周期状态（积分抗饱和回算）
         * @details 传入预计算结果 + 经过电压限幅后的有效输出，通过抗饱和算法修正积分贡献；
         *          输出待提交的下一状态，**不直接写入PI内部状态**，需调用commit生效。
         * @param[in] preview 上一步preview得到的预计算数据包
         * @param[in] limited_output 经过电压限幅模块约束后的有效αβ电压
         * @param[out] next_state_out 修正后的PI下一状态（积分值等）
         * @retval ErrorCode::Ok 状态准备完成
         * @retval 其他错误码：参数非法
         */
        [[nodiscard]] common::ErrorCode prepare_update(
            const PiPreview& preview,
            common::Volts limited_output,
            PiNextState& next_state_out
        ) const noexcept;

        /**
         * @brief 提交并固化PI下一周期状态
         * @details 将prepare_update算出的新状态写入PI内部，完成一次闭环步进；
         * @param[in] next_state 由prepare_update输出的待生效状态
         */
        void commit(
            const PiNextState& next_state
        ) noexcept;

        /**
         * @brief 设置PI参数并校验配置合法性
         * @param[in] config PI配置参数（Kp、Ki、积分限幅等）
         * @retval ErrorCode::Ok 参数合法，配置生效
         * @retval 其他错误码：参数非法（增益<=0、限幅范围错误等）
         */
        [[nodiscard]] common::ErrorCode configure(const PiConfig& config) noexcept;
        
        /**
         * @brief 重置PI运行时状态
         * @details 清零积分缓存、误差历史等动态状态；
         * @note 保留原有configure的增益配置不变
         */
        void reset() noexcept;
    };
    
    /**
     * @brief 位置式Pid控制器
     */
    class PositionalPid final
    {
    private:
        float kp_{0.0F};  //比例系数
        float ki_{0.0F};  //积分系数
        float kd_{0.0F};  //微分系数

        float out_min_{0.0F};  //输出下限
        float out_max_{0.0F};  //输出上限
        float integral_min_{0.0F};  //积分项下限
        float integral_max_{0.0F};  //积分项上限

        float integral_{0.0F};  //积分累加值
        float last_err_{0.0F};  //上一次误差

    public:
        /**
         * @brief 构造位置式PID
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param out_min 输出下限
         * @param out_max 输出上限
         * @param integral_min 积分项下限
         * @param integral_max 积分项上限
         */
        PositionalPid(float kp, float ki, float kd, float out_min,
            float out_max, float integral_min, float integral_max) noexcept;

        // 禁止拷贝、禁止移动，防止无意复制控制器状态
        PositionalPid(const PositionalPid&) = delete;
        PositionalPid& operator=(const PositionalPid&) = delete;
        PositionalPid(PositionalPid&&) = delete;
        PositionalPid& operator=(PositionalPid&&) = delete;

        /**
         * @brief 单次PID计算
         * @param setpoint 目标设定值
         * @param feedback 反馈测量值
         * @return 位置式控制输出，做输出限幅
         */
        [[nodiscard]] float calculate(float setpoint, float feedback) noexcept;

        /**
         * @brief 复位，清零积分项与历史误差
         */
        void reset() noexcept;

        /**
         * @brief 动态修改Pid参数
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         */
        void set_param(float kp, float ki, float kd) noexcept;

        /**
         * @brief 获取kp
         * @return kp_
         */
        [[nodiscard]] float get_kp() const noexcept;

        /**
         * @brief 获取ki
         * @return ki_
         */
        [[nodiscard]] float get_ki() const noexcept;

        /**
         * @brief 获取kd
         * @return kd_
         */
        [[nodiscard]] float get_kd() const noexcept;

        /**
         * @brief 获取积分累积值
         * @return integral_
         */
        [[nodiscard]] float get_integral() const noexcept;

        /**
         * @brief 获取上一次误差
         * @return last_err_
         */
        [[nodiscard]] float get_last_err() const noexcept;
    };

    /**
     * @brief 增量式Pid控制器
     */
    class IncrementalPid final
    {
    private:
        float kp_{0.0F};  //比例系数
        float ki_{0.0F};  //积分系数
        float kd_{0.0F};  //微分系数

        float delta_out_min_{0.0F};  //增量输出下限
        float delta_out_max_{0.0F};  //增量输出上限

        float err1_{0.0F};  //e(k)
        float err2_{0.0F};  //e(k-1)
        float err3_{0.0F};  //e(k-2)

    public:
        /**
         * @brief 构造增量式PID
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param delta_out_min 输出增量下限
         * @param delta_out_max 输出增量上限
         */
        IncrementalPid(float kp ,float ki, float kd,
            float delta_out_min, float delta_out_max
        ) noexcept;
        ~IncrementalPid() noexcept = default;

        // 禁止拷贝、禁止移动，防止无意复制控制器状态
        IncrementalPid(const IncrementalPid&) = delete;
        IncrementalPid& operator=(const IncrementalPid&) = delete;
        IncrementalPid(IncrementalPid&&) = delete;
        IncrementalPid& operator=(IncrementalPid&&) = delete;

        /**
         * @brief 单次PID计算
         * @param setpoint 目标设定值
         * @param feedback 反馈测量值
         * @return 控制增量Δu，做输出限幅
         */
        [[nodiscard]] float calculate(float setpoint, float feedback) noexcept;

        /**
         * @brief 复位，err1,err2,err3清零
         */
        void reset() noexcept;

        /**
         * @brief 动态修改Pid参数
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         */
        void set_param(float kp, float ki, float kd) noexcept;

        /**
         * @brief 获取kp
         * @return kp_
         */
        [[nodiscard]] float get_kp() const noexcept;

        /**
         * @brief 获取ki
         * @return ki_
         */
        [[nodiscard]] float get_ki() const noexcept;

        /**
         * @brief 获取kd
         * @return kd_
         */
        [[nodiscard]] float get_kd() const noexcept;

        /**
         * @brief 获取err1
         * @return err1_
         */
        [[nodiscard]] float get_err1() const noexcept;

        /**
         * @brief 获取err2
         * @return err2_
         */
        [[nodiscard]] float get_err2() const noexcept;

        /**
         * @brief 获取err3
         * @return err3_
         */
        [[nodiscard]] float get_err3() const noexcept;



    };

} // namespace robot_foc::algorithm

#endif
