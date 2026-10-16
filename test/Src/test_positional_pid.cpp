/**
 * @brief 位置式PID算法单元测试
 * @details 验证位置式PID算法输出限幅、积分限幅、复位及阶跃响应积分饱和功能
 */

#include "test_helper.hpp"
#include "ALGORITHM/pid.hpp"

/**
 * @brief 位置式PID抗积分饱和单元测试
 * @details 验证方向感知抗饱和积分逻辑：
 *  1. 饱和积分 + 同向误差：禁止积分累计
 *  2. 饱和积分 + 反向误差：允许积分减小，实现退饱和
 *  3. 积分限幅兜底保护
 */
bool test_positional_pid()
{
    using robot_foc::algorithm::PositionalPid;

    PositionalPid pid
    (
        1.0F,
        1.0F,
        0.0F,
        -1.0F,
        1.0F,
        -5.0F,
        5.0F
    );

    pid.reset();
    
}




