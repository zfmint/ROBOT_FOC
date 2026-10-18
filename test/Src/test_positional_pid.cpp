/**
 * @brief 位置式PID算法单元测试
 * @details 验证位置式PID算法输出限幅、积分限幅、复位及阶跃响应积分饱和功能
 */

#include <iostream>
#include <cmath>

#include "test_helper.hpp"
#include "ALGORITHM/pid.hpp"
#include "mock_plant.hpp"

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
    
    //reset 校验
    pid.reset();
    float integral_after_reset = pid.get_integral();
    float last_err_after_reset = pid.get_last_err();

    EXPECT(fabsf(integral_after_reset - 0.0F) < 1e-6F,"PositionalPID reset integral clear");
    EXPECT(fabsf(last_err_after_reset - 0.0F) < 1e-6F,"PositionalPID reset last_err clear");

    //饱和 + 同向误差，禁止积分累计
    pid.reset();
    const float set_1 = 10.0F;
    const float fb_1 = 0.0F;

    (void)pid.calculate(set_1,fb_1);
    const float integral_1 = pid.get_integral();

    (void)pid.calculate(set_1,fb_1);
    const float integral_2 = pid.get_integral();

    /**
     * 逻辑预期
     * 预输出 >= outmax(1.0F),err>0,禁止积分累加
     * integral_2不能明显大于integral_1
     * 允许微小浮点误差，加1e-6容差
     */
    EXPECT((integral_2 <= integral_1 + 1e-6F),"saturated same direction, stop integral accumulate");
    
    
    // ========== 场景3：饱和 + 反向误差，允许积分回落退饱和 ==========
    pid.reset();
    // Step1：先建立正向饱和工况，把积分积累起来
    const float set_sat = 10.0F;
    const float fb_sat = 0.0F;
    (void)pid.calculate(set_sat, fb_sat);
    (void)pid.calculate(set_sat, fb_sat);
    const float integral_before_reverse = pid.get_integral();
    std::cout << "integral_before_reverse: " << integral_before_reverse << "\n";

    // Step2：不reset！构造【仍然正向饱和 + 误差反向】
    const float set_2 = 10.0F;
    const float fb_2 = 10.1F; // 关键修改！fb只比给定大一点点，不要拉到15！
    auto out = pid.calculate(set_2,fb_2);
    std::cout << "out: " << out << "\n";
    const float integral_3 = pid.get_integral();
    std::cout << "integral_3: " << integral_3 << "\n";

    EXPECT( (integral_3 < integral_before_reverse + 1e-6F),"saturated reverse error,integral decrease (anti-windup work)");
    
    //积分限幅兜底
    //构造不饱和工况，让积分持续累积直到饱和
    for (int i = 0; i < 100; i++)
    {
        (void)pid.calculate(10.0F,0.0F);
    }
    const float integral_final = pid.get_integral();

    EXPECT((integral_final <= 5.0F + 1e-6F),"integral upper limit clamp");
    EXPECT((integral_final >= -5.0F - 1e-6F),"integral lower limit clamp");

    return true;
}

/**
 * @brief 位置式PID闭环阶跃响应测试
 * @details 配合一阶惯性被控对象，验证阶跃跟踪能力、超调
 */
bool test_positional_pid_closed()
{
    using robot_foc::algorithm::PositionalPid;
    using robot_foc::test::MockFirstOrderPlant;

    const float Ts = 0.01F;
    const float tau = 0.2F;
    const float set = 5.0F;
    const std::size_t sim_steps = 300;

    PositionalPid pid
    {
        1.0F,
        1.0F,
        0.0F,
        -1.0F,
        1.0F,
        -5.0F,
        5.0F 
    };
    robot_foc::test::MockFirstOrderPlant plant(Ts,tau);

    pid.reset();
    plant.reset();

    float fb = 0.0F;
    float fb_max = 0.0F;

    for (size_t i = 0; i < sim_steps; i++)
    {
        const float u = pid.calculate(set,fb);
        fb = plant.update(u);

        if (fb > fb_max)
        {
            fb_max = fb;
        }
    }

    const float steady_tol = 0.1F;
    EXPECT(fabs(fb - set) < steady_tol,"closed step response: steady state error within tolerance");
    
    const float upper_limit = 5.3F;
    EXPECT(fb_max < upper_limit,"closed step response: overshoot is controlled");

    return true;
}
