/**
 * @brief 增量式PID算法单元测试
 * @details 验证增量式PIDΔu数学计算、增量限幅、复位及闭环阶跃响应特性
 */

#include <iostream>
#include <cmath>

#include "test_helper.hpp"
#include "ALGORITHM/pid.hpp"
#include "mock_plant.hpp"

/**
 * @brief 增量PID基础单元测试
 * @details 验证增量计算公式正确性、reset复位、增量Δu限幅功能
 * 增量公式：Δu = Kp*(e_k - e_{k-1}) + Ki*e_k + Kd*(e_k - 2*e_{k-1}+e_{k-2})
 */
void test_incremental_pid()
{
    using robot_foc::algorithm::IncrementalPid;

    // 基础数学计算：纯比例 Kp only
    {
        IncrementalPid pid_pureP
        {
            2.0F,
            0.0F,
            0.0F,
            -10.0F,
            10.0F
        };
        pid_pureP.reset();

        float du1 = pid_pureP.calculate(1.0F, 0.0F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du1 - 2.0F) < 1e-6F,"inc pid pureP calc du1");

        float du2 = pid_pureP.calculate(1.0F, 0.2F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du2 - (-0.4F)) < 1e-6F, "inc pid pureP calc du2");

        float du3 = pid_pureP.calculate(1.0F, 0.5F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du3 - (-0.6F)) < 1e-6F, "inc pid pureP calc du3");
    }

    // Kp + Ki，无微分项
    {
        IncrementalPid pid_pi
        {
            1.0F,
            0.5F,
            0.0F,
            -10.0F,
            10.0F
        };
        pid_pi.reset();

        float du1 = pid_pi.calculate(1.0F, 0.0F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du1 - 1.5F) < 1e-6F,"inc pid PI calc du1");

        float du2 = pid_pi.calculate(1.0F, 0.5F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du2 - (-0.25F)) < 1e-6F, "inc pid PI calc du2");
    }

    // Kp+Ki+Kd 三项全部启用，完整公式校验
    {
        IncrementalPid pid_full
        {
            2.0F,
            0.4F,
            0.1F,
            -10.0F,
            10.0F
        };
        pid_full.reset();

        float du1 = pid_full.calculate(1.0F, 0.0F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du1 - 2.5F) < 1e-6F,"inc pid full 3-term calc du1");

        float du2 = pid_full.calculate(1.0F, 0.3F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du2 - (-0.45F)) < 1e-6F, "inc pid full 3-term calc du2");

        float du3 = pid_full.calculate(1.0F, 0.6F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du3 - (-0.44F)) < 1e-6F, "inc pid full 3-term calc du3");
    }

    // reset 测试：清空三段历史误差 e_k,e_{k-1},e_{k-2}
    {
        IncrementalPid pid_reset
        {
            1.0F,
            0.2F,
            0.0F,
            -10.0F,
            10.0F
        };
        pid_reset.reset();

        (void)pid_reset.calculate(1.0F, 0.0F);
        pid_reset.reset();

        // reset后历史误差清零，第一拍计算结果和全新实例一致
        float du = pid_reset.calculate(1.0F, 0.0F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du - 1.2F) < 1e-6F, "inc pid reset clear error history");
    }

    // Δu增量限幅测试
    {
        IncrementalPid pid_du_clamp
        {
            10.0F,
            0.0F,
            0.0F,
            -1.0F,
            1.0F
        };
        pid_du_clamp.reset();

        float du_upper = pid_du_clamp.calculate(1.0F,0.0F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du_upper - 1.0F) <1e-6F,"inc pid du upper limit");

        float du_lower = pid_du_clamp.calculate(-1.0F,0.0F);
        ROBOT_FOC_TEST_EXPECT(std::fabs(du_lower - (-1.0F)) <1e-6F,"inc pid du lower limit");
    }

}

/**
 * @brief 增量式PID闭环阶跃响应测试
 * @details 配合一阶惯性被控对象，验证阶跃跟踪能力、超调；
 * 增量式PID输出Δu，外部累加得到总控制量u，并在外部实现抗饱和逻辑
 */
void test_incremental_pid_closed()
{
    using robot_foc::algorithm::IncrementalPid;
    using robot_foc::test::MockFirstOrderPlant;

    const float Ts = 0.01F;
    const float tau = 0.2F;
    const float set = 1.0F;
    const std::size_t sim_steps = 300;
    const float max_overshoot_ratio = 0.15F; // 允许最大15%超调
    const float out_max = 1.0F;
    const float out_min = -1.0F;

    IncrementalPid pid
    {
        1.2F,
        2.5F,
        0.0F,
        out_min,
        out_max
    };
    robot_foc::test::MockFirstOrderPlant plant(Ts,tau);

    pid.reset();
    plant.reset();

    float fb = 0.0F;
    float fb_max = 0.0F;
    float u_prev = 0.0F; //增量PID：外部维护上一拍总输出u

    for (size_t i = 0; i < sim_steps; i++)
    {
        const float delta_u = pid.calculate(set,fb);
        const float u_candidate = u_prev + delta_u;
        float u;

        //增量式外部抗饱和逻辑
        if (u_candidate > out_max)
        {
            if (delta_u > 0.0F)
            {
                //正向饱和，增量继续增大，丢弃增量，锁上限
                u = out_max;
            }
            else
            {
                //增量反向，允许退饱和
                u = u_candidate;
            }
        }
        else if (u_candidate < out_min)
        {
            if (delta_u < 0.0F)
            {
                //负向饱和，继续向下，锁下限
                u = out_min;
            }
            else
            {
                u = u_candidate;
            }
        }
        else
        {
            u = u_candidate;
        }

        fb = plant.update(u);
        u_prev = u;

        if (fb > fb_max)
        {
            fb_max = fb;
        }
    }

    // 稳态误差校验
    const float steady_tol = 0.1F;
    ROBOT_FOC_TEST_EXPECT(std::fabs(fb - set) < steady_tol,"inc closed step response: steady state error within tolerance");

    // 超调校验：峰值不超过设定值*(1+超调比例)，加浮点容差
    const float peak_max_allowed = set * (1.0F + max_overshoot_ratio);
    ROBOT_FOC_TEST_EXPECT(fb_max <= peak_max_allowed + 1e-6F, "inc closed step response: overshoot is controlled");

    std::cout << "\n[Incremental Closed Step] set=" << set
              << " | peak_fb=" << fb_max
              << " | final_fb=" << fb
              << " | allowed_peak=" << peak_max_allowed << "\n";

}
