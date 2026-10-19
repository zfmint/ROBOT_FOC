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
    
    //输出限幅测试
    {
        PositionalPid pid_pureP
        {
            1.0F,
            0.0F,
            0.0F,
            -1.0F,
            1.0F,
            -5.0F,
            5.0F
        };
        pid_pureP.reset();

        float out_upper = pid_pureP.calculate(10.0F,0.0F);
        EXPECT(fabsf(out_upper - 1.0F) < 1e-6F,"output limit upper bound");

        float out_lower = pid_pureP.calculate(-10.0F,0.0F);
        EXPECT(fabsf(out_lower - (-1.0F)) < 1e-6F, "output limit lower bound");
    }

    //reset 测试
    {
        PositionalPid pid_reset
        {
            1.0F,
            1.0F,
            0.0F,
            -1.0F,
            1.0F,
            -5.0F,
            5.0F
        };

        (void)pid_reset.calculate(0.5F, 0.0F);
        float int_before_rst = pid_reset.get_integral();
        float err_before_rst = pid_reset.get_last_err();
        EXPECT(int_before_rst > 1e-6F, "reset prepare: integral not zero");
        EXPECT(err_before_rst > 1e-6F, "reset prepare: last_err not zero");

        pid_reset.reset();
        float int_after_rst = pid_reset.get_integral();
        float err_after_rst = pid_reset.get_last_err();
        EXPECT(fabsf(int_after_rst) < 1e-6F, "reset clear integral");
        EXPECT(fabsf(err_after_rst) < 1e-6F, "reset clear last_err");
    }

    //积分限幅兜底测试
    {
        PositionalPid pid_int_clamp
        (
            0.0F,    // Kp=0，无比例项
            1.0F,    // Ki=1
            0.0F,    // Kd=0
            -100.0F, // 输出限幅极大，永远不会饱和，抗饱和判定永远不触发
            100.0F,
            -5.0F,   // 积分下限
            5.0F     // 积分上限
        );
        pid_int_clamp.reset();
        // 持续正向误差 e=1.0，积分持续累加，最终被钳到+5
        for(int i=0; i<100; i++)
        {
            (void)pid_int_clamp.calculate(1.0F, 0.0F);
        }
        const float int_upper = pid_int_clamp.get_integral();
        EXPECT(fabsf(int_upper - 5.0F) < 1e-6F, "integral upper clamp to 5");

        pid_int_clamp.reset();
        // 持续负向误差 e=-1.0，积分持续累加，最终被钳到-5
        for(int i=0; i<100; i++)
        {
            (void)pid_int_clamp.calculate(-1.0F, 0.0F);
        }
        const float int_lower = pid_int_clamp.get_integral();
        EXPECT(fabsf(int_lower - (-5.0F)) < 1e-6F, "integral lower clamp to -5");
    }

    //抗积分饱和测试
    {
        PositionalPid pid_antiwindup
        (
            0.0F,
            1.0F,
            0.0F,
            -1.0F,
            1.0F,
            -5.0F,
            5.0F
        );
        pid_antiwindup.reset();

        // Step1: 第1拍，积分=0.5，pre_out=0，不饱和
        (void)pid_antiwindup.calculate(0.5F, 0.0F);
        std::cout << "After calc1, int = " << pid_antiwindup.get_integral() << "\n";

        // Step2: 第2拍，积分涨到1.0；pre_out=0.5，仍然不饱和！
        (void)pid_antiwindup.calculate(0.5F, 0.0F);
        std::cout << "After calc2, int = " << pid_antiwindup.get_integral() << "\n";

        // Step3: 第3拍！pre_out = 1.0，触发正向饱和，err=0.5同向 → 禁止积分累加
        (void)pid_antiwindup.calculate(0.5F, 0.0F);
        std::cout << "After calc3(saturated), int = " << pid_antiwindup.get_integral() << "\n";
        const float integral_saturated = pid_antiwindup.get_integral();
        EXPECT(fabsf(integral_saturated -1.0F) <1e-6F, "anti-windup: integral reach saturation point");

        // Step4：继续送入同向误差 e=0.5，保持饱和+同向误差 → 积分冻结，维持1.0
        (void)pid_antiwindup.calculate(0.5F, 0.0F);
        std::cout << "After calc4(same dir), int = " << pid_antiwindup.get_integral() << "\n";
        const float integral_same_dir = pid_antiwindup.get_integral();
        EXPECT(integral_same_dir <= integral_saturated + 1e-6F, "anti-windup: saturated same direction stop accumulate");

        // Step5：送入反向误差 e=-0.2，饱和状态下允许积分下降
        (void)pid_antiwindup.calculate(-0.2F, 0.0F);
        std::cout << "After calc5(reverse), int = " << pid_antiwindup.get_integral() << "\n";
        const float integral_rev_dir = pid_antiwindup.get_integral();
        // 预期：1.0 -0.2 = 0.8
        EXPECT(fabsf(integral_rev_dir - 0.8F) <1e-6F, "anti-windup: saturated reverse error integral decrease");
    }

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
    const float set = 1.0F;
    const std::size_t sim_steps = 300;
    const float max_overshoot_ratio = 0.15F; // 允许最大15%超调

    PositionalPid pid
    {
        1.2F,
        2.5F,
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

    // 稳态误差校验
    const float steady_tol = 0.1F;
    EXPECT(fabsf(fb - set) < steady_tol,"closed step response: steady state error within tolerance");

    // 超调校验：峰值不超过设定值*(1+超调比例)，加浮点容差
    const float peak_max_allowed = set * (1.0F + max_overshoot_ratio);
    EXPECT(fb_max <= peak_max_allowed + 1e-6F, "closed step response: overshoot is controlled");

    std::cout << "\n[Closed Step] set=" << set
              << " | peak_fb=" << fb_max
              << " | final_fb=" << fb
              << " | allowed_peak=" << peak_max_allowed << "\n";

    return true;
}
