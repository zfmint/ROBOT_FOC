/**
 * @brief 分步式PiController单元测试
 * @details
 * 调用链路：preview(预计算，只读不修改内部状态)
 *        → 外部限幅
 *        → update_after_limit(条件积分冻结并原子提交下一状态)
 * 抗饱和策略：条件积分冻结；限幅且误差加剧饱和时冻结本拍积分累加；误差反转允许退饱和。
 * 覆盖：配置校验、非法配置不污染旧状态、reset、正常积分累加、正负向抗饱和冻结、积分限幅；
 *      NaN/Inf非法输入、失败不篡改输出参数、未配置保护；
 *      闭环阶跃仿真（MockFirstOrderPlant一阶惯性对象）。
 */
#include <iostream>
#include <cmath>
#include "test_helper.hpp"
#include "ALGORITHM/pid.hpp"
#include "mock_plant.hpp"

/**
 * @brief 分步PI基础单元测试
 */
void test_pi_controller()
{
    using robot_foc::algorithm::PiController;
    using robot_foc::algorithm::PiConfig;
    using robot_foc::algorithm::PiPreview;
    using robot_foc::common::Amperes;
    using robot_foc::common::Volts;
    using robot_foc::common::Seconds;
    using robot_foc::common::ErrorCode;

    // ========== 测试1：正常无饱和，积分逐步累加 ==========
    {
        PiController pi{};
        PiConfig cfg{};
        cfg.kp_ = 1.0F;
        cfg.ki_ = 100.0F;
        cfg.sample_time_ = Seconds{0.001F};
        cfg.integral_min_ = -20.0F;
        cfg.integral_max_ = 20.0F;
        auto ret_cfg = pi.configure(cfg);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi configure valid ok");
        pi.reset();
        PiPreview pre{};
        // 第1周期
        auto ret_pre = pi.preview(Amperes{2.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi step1 preview ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(pre.error_, 2.0F), "pi step1 error");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(pre.request_output_), 2.0F), "pi step1 request output");

        auto ret_update = pi.update_after_limit(pre, Volts{2.0F}, false);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi step1 update ok");
        // 第2周期
        ret_pre = pi.preview(Amperes{2.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi step2 preview ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(pre.request_output_), 2.2F), "pi step2 request output");
    }

    // ========== 测试2：正向饱和-条件积分冻结 ==========
    {
        PiController pi{};
        PiConfig cfg{};
        cfg.kp_ = 1.0F;
        cfg.ki_ = 100.0F;
        cfg.sample_time_ = Seconds{0.001F};
        cfg.integral_min_ = -20.0F;
        cfg.integral_max_ = 20.0F;
        auto ret_cfg = pi.configure(cfg);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi anti-windup configure ok");
        pi.reset();
        PiPreview pre{};
        auto ret_pre = pi.preview(Amperes{5.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi anti-windup preview ok");
        auto ret_update = pi.update_after_limit(pre, Volts{3.0F}, true);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi anti-windup update ok");
        ret_pre = pi.preview(Amperes{5.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi anti-windup verify preview ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(pre.request_output_, 5.0F), "pi positive saturate freeze integral");
    }

    // ========== 测试3：负向饱和-条件积分冻结 ==========
    {
        PiController pi{};
        PiConfig cfg{};
        cfg.kp_ = 1.0F;
        cfg.ki_ = 100.0F;
        cfg.sample_time_ = Seconds{0.001F};
        cfg.integral_min_ = -20.0F;
        cfg.integral_max_ = 20.0F;
        auto ret_cfg = pi.configure(cfg);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi neg anti-windup configure ok");
        pi.reset();
        PiPreview pre{};
        auto ret_pre = pi.preview(Amperes{-5.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi neg anti-windup preview ok");
        auto ret_update = pi.update_after_limit(pre, Volts{-3.0F}, true);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi neg anti-windup update ok");
        ret_pre = pi.preview(Amperes{-5.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi neg anti-windup verify preview ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(pre.request_output_, -5.0F), "pi negative saturate freeze integral");
    }

    // ========== 测试4：积分上下限钳位 ==========
    {
        PiController pi{};
        PiConfig cfg{};
        cfg.kp_ = 0.0F;
        cfg.ki_ = 10000.0F;
        cfg.sample_time_ = Seconds{0.001F};
        cfg.integral_min_ = -5.0F;
        cfg.integral_max_ = 5.0F;
        auto ret_cfg = pi.configure(cfg);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi integral clamp configure ok");
        pi.reset();
        PiPreview pre{};
        for(int i = 0; i < 10; ++i)
        {
            auto ret_pre = pi.preview(Amperes{1.0F}, Amperes{0.0F}, pre);
            ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi clamp loop preview ok");
            auto ret_update = pi.update_after_limit(pre, Volts{0.0F}, false);
            ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi clamp loop update ok");
        }
        auto ret_pre = pi.preview(Amperes{0.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi upper clamp verify preview ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(pre.request_output_, 5.0F), "pi integral clamp upper limit");

        pi.reset();
        for(int i = 0; i < 10; ++i)
        {
            ret_pre = pi.preview(Amperes{-1.0F}, Amperes{0.0F}, pre);
            ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi neg clamp loop preview ok");
            auto ret_update = pi.update_after_limit(pre, Volts{0.0F}, false);
            ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi neg clamp loop update ok");
        }
        ret_pre = pi.preview(Amperes{0.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi lower clamp verify preview ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(pre.request_output_, -5.0F), "pi integral clamp lower limit");
    }

    // ========== 测试5：reset复位清零积分 ==========
    {
        PiController pi{};
        PiConfig cfg{};
        cfg.kp_ = 1.0F;
        cfg.ki_ = 100.0F;
        cfg.sample_time_ = Seconds{0.001F};
        cfg.integral_min_ = -10.0F;
        cfg.integral_max_ = 10.0F;
        auto ret_cfg = pi.configure(cfg);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi reset configure ok");

        PiPreview pre{};
        for(int i=0;i<5;i++)
        {
            auto ret_pre = pi.preview(Amperes{3.0F}, Amperes{0.0F}, pre);
            ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi reset-fill preview ok");
            auto ret_update = pi.update_after_limit(pre, Volts{0.0F}, false);
            ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi reset-fill update ok");
        }

        pi.reset();
        auto ret_pre = pi.preview(Amperes{0.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi reset after preview ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(pre.request_output_), 0.0F), "pi reset clear integral_output");
    }

    // ========== 测试6：configure 非法配置：拒绝生效，保留旧配置与旧状态 ==========
    {
        PiController pi{};
        PiConfig good_cfg{};
        good_cfg.kp_ = 2.0F;
        good_cfg.ki_ = 80.0F;
        good_cfg.sample_time_ = Seconds{0.001F};
        good_cfg.integral_min_ = -12.0F;
        good_cfg.integral_max_ = 12.0F;
        auto ret_ok = pi.configure(good_cfg);
        ROBOT_FOC_TEST_EXPECT(ret_ok == ErrorCode::Ok, "pi valid config apply ok");

        // 跑一轮，制造积分状态
        PiPreview pre{};
        auto ret_pre = pi.preview(Amperes{2.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi pre-badcfg preview ok");
        auto ret_update = pi.update_after_limit(pre, Volts{0}, false);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi pre-badcfg update ok");

        // 传入非法配置，应当返回错误，**不覆盖原有参数、不清除现有积分状态**
        PiConfig bad_cfg = good_cfg;
        bad_cfg.kp_ = -10.0F;
        auto ret_bad = pi.configure(bad_cfg);
        ROBOT_FOC_TEST_EXPECT(ret_bad == ErrorCode::ErrorInvalidParam, "pi reject bad config");

        // 继续运行，确认旧配置、旧积分仍然生效
        ret_pre = pi.preview(Amperes{0.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi use old config after bad configure");
        // request_output = kp(旧2.0)*0 + 上一轮积分，不等于0，证明积分没有被清零
        ROBOT_FOC_TEST_EXPECT(std::fabs(static_cast<float>(pre.request_output_)) > 0.001F, "pi bad configure keep old integral state");
    }

    // ========== 测试7：未配置状态保护 preview / update_after_limit ==========
    {
        using robot_foc::algorithm::PiPreview;
        PiController pi{};
        PiPreview pre{};    // {} 值初始化，全部成员置0
        auto ret_pre = pi.preview(Amperes{1.0F}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::ErrorNotReady, "pi preview before configure reject");
        auto ret_update = pi.update_after_limit(pre, Volts{0.0F}, false);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::ErrorNotReady, "pi update before configure reject");
    }

    // ========== 测试8：NaN / Inf输入 preview；输出参数不篡改（保留脏值） ==========
    {
        PiController pi{};
        PiConfig cfg{};
        cfg.kp_ =1.0F;
        cfg.ki_ = 100.0F;
        cfg.sample_time_ = Seconds{0.001F};
        cfg.integral_min_ = -10;
        cfg.integral_max_ = 10;
        auto ret_cfg = pi.configure(cfg);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi nan-inf configure ok");

        PiPreview pre{};
        // 预先写入脏值，失败不修改输出结构体
        pre.error_ = 999.0F;
        pre.request_output_ = Volts{888.0F};

        auto ret_nan = pi.preview(Amperes{NAN}, Amperes{0.0F}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_nan == ErrorCode::ErrorInvalidParam, "pi preview nan setpoint reject");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(pre.error_, 999.0F), "pi preview fail preserve pre.error");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(pre.request_output_), 888.0F), "pi preview fail preserve pre.request_output");

        pre.error_ = 777.0F;
        pre.request_output_ = Volts{666.0F};
        auto ret_inf = pi.preview(Amperes{1.0F}, Amperes{INFINITY}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_inf == ErrorCode::ErrorInvalidParam, "pi preview inf feedback reject");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(pre.error_,777.0F), "pi preview inf preserve error");
    }

    // ========== 测试9：update_after_limit非法输入不篡改内部积分状态 ==========
    {
        PiController pi{};
        PiConfig cfg{};
        cfg.kp_ = 1.0F;
        cfg.ki_ = 100.0F;
        cfg.sample_time_ = Seconds{0.001F};
        cfg.integral_min_ = -10;
        cfg.integral_max_ = 10;
        auto ret_cfg = pi.configure(cfg);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi prepare nan configure ok");

        PiPreview valid_pre{};
        auto ret_pre = pi.preview(Amperes{1.0F}, Amperes{0.0F}, valid_pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi prepare valid state preview ok");
        auto ret_update = pi.update_after_limit(valid_pre, Volts{1.0F}, false);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi prepare valid state update ok");

        PiPreview bad_pre{};
        bad_pre.error_ = NAN;
        bad_pre.request_output_ = Volts{5.0F};
        ret_update = pi.update_after_limit(bad_pre, Volts{3.0F}, true);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::ErrorInvalidParam, "pi update detect nan in preview");

        PiPreview verify_pre{};
        ret_pre = pi.preview(Amperes{0.0F}, Amperes{0.0F}, verify_pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi verify state after failed update");
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(verify_pre.request_output_, 0.1F),
            "pi failed update preserves integral state"
        );
    }
}

/**
 * @brief 分步式PI闭环阶跃响应测试
 * @details
 * 仿真流程：preview得到原始输出 → 外部限幅得到limited_out与limited_flag → update_after_limit更新PI
 * 将PI输出电压送给一阶惯性MockPlant，验证阶跃跟踪、稳态误差与超调
 */
void test_pi_controller_closed()
{
    using robot_foc::algorithm::PiController;
    using robot_foc::test::MockFirstOrderPlant;
    using robot_foc::common::Amperes;
    using robot_foc::common::Volts;
    using robot_foc::common::Seconds;
    using robot_foc::common::ErrorCode;

    const float Ts = 0.01F;
    const float tau = 0.2F;
    const float set_current = 1.0F;
    const std::size_t sim_steps = 300;
    const float max_overshoot_ratio = 0.15F;
    const float voltage_limit = 1.0F;

    PiController pi{};
    robot_foc::algorithm::PiConfig cfg{};
    cfg.kp_ = 1.2F;
    cfg.ki_ = 2.5F;
    cfg.sample_time_ = Seconds{Ts};
    cfg.integral_min_ = -5.0F;
    cfg.integral_max_ = 5.0F;
    auto ret_cfg = pi.configure(cfg);
    ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "pi closed configure ok");
    pi.reset();

    MockFirstOrderPlant plant(Ts, tau);
    plant.reset();

    float fb = 0.0F;
    float fb_max = 0.0F;

    for (size_t i = 0; i < sim_steps; i++)
    {
        robot_foc::algorithm::PiPreview pre{};
        auto ret_pre = pi.preview(Amperes{set_current}, Amperes{fb}, pre);
        ROBOT_FOC_TEST_EXPECT(ret_pre == ErrorCode::Ok, "pi closed preview ok");

        float raw_u = static_cast<float>(pre.request_output_);
        float limited_u;
        bool limited_flag = false;
        if(raw_u > voltage_limit)
        {
            limited_u = voltage_limit;
            limited_flag = true;
        }
        else if(raw_u < -voltage_limit)
        {
            limited_u = -voltage_limit;
            limited_flag = true;
        }
        else
        {
            limited_u = raw_u;
        }

        auto ret_update = pi.update_after_limit(pre, Volts{limited_u}, limited_flag);
        ROBOT_FOC_TEST_EXPECT(ret_update == ErrorCode::Ok, "pi closed update_after_limit ok");

        fb = plant.update(limited_u);
        if(fb > fb_max)
        {
            fb_max = fb;
        }
    }

    const float steady_tol = 0.1F;
    ROBOT_FOC_TEST_EXPECT(std::fabs(fb - set_current) < steady_tol,
        "pi closed step response: steady state error within tolerance");

    const float peak_max_allowed = set_current * (1.0F + max_overshoot_ratio);
    ROBOT_FOC_TEST_EXPECT(fb_max <= peak_max_allowed + 1e-6F,
        "pi closed step response: overshoot is controlled");

    std::cout << "\n[PiController Closed Step] set=" << set_current
              << " | peak_fb=" << fb_max
              << " | final_fb=" << fb
              << " | allowed_peak=" << peak_max_allowed << "\n";
}
