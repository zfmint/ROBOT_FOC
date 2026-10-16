/**
 * @file pid.hpp
 * @brief Pid算法定义。提供两种Pid算法：位置式Pid、增量式Pid
 * @details
 *  用途： 通用PID控制算法，用于闭环控制
 *  特性：
 *      1. 位置式PID PositionalPid
 *          - 输入：设定值setpoint，反馈值feedback
 *          - 参数：kp，ki，kd
 *          - 功能：完整积分项，支持积分限幅、输出限幅，reset()清零积分与历史误差状态
 *      2. 增量式PID IncrementalPid
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

namespace robot_foc::algorithm
{
    /**
     * @brief 位置式Pid控制器
     */
    class PositionalPid
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
        PositionalPid(float kp, float ki, float kd, float out_min,
            float out_max, float integral_min, float integral_max) noexcept;

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
        [[nodiscard]] float get_kp()const noexcept;

        /**
         * @brief 获取ki
         * @return ki_
         */
        [[nodiscard]] float get_ki()const noexcept;

        /**
         * @brief 获取kd
         * @return kd_
         */
        [[nodiscard]] float get_kd()const noexcept;

        /**
         * @brief 获取积分累积值
         * @return integral_
         */
        [[nodiscard]] float get_integral()const noexcept;

        /**
         * @brief 获取上一次误差
         * @return last_err_
         */
        [[nodiscard]] float get_last_err()const noexcept;
    };
    
    /**
     * @brief 增量式Pid控制器
     */
    class IncrementalPid
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
        IncrementalPid(float kp ,float ki, float kd,
            float delta_out_min, float delta_out_max) noexcept;

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
        [[nodiscard]] float get_kp()const noexcept;

        /**
         * @brief 获取ki
         * @return ki_
         */
        [[nodiscard]] float get_ki()const noexcept;

        /**
         * @brief 获取kd
         * @return kd_
         */
        [[nodiscard]] float get_kd()const noexcept;

        /**
         * @brief 获取err1
         * @return err1_
         */
        [[nodiscard]] float get_err1()const noexcept;

        /**
         * @brief 获取err2
         * @return err2_
         */
        [[nodiscard]] float get_err2()const noexcept;

        /**
         * @brief 获取err3
         * @return err3_
         */
        [[nodiscard]] float get_err3()const noexcept;



    };
    
} // namespace robot_foc::algorithm

#endif
