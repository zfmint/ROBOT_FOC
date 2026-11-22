/**
 * @file test_pi_controller.cpp
 * @brief PiController分步计算、抗积分饱和和闭环响应单元测试。
 * @details
 * PiController的prepare_update()和commit()仅供CurrentLoop使用。测试通过无状态、
 * 无运行时开销的PiControllerTestAccess访问器验证同一条内部调用路径，不扩大生产接口。
 */

#include <cmath>
#include <cstddef>
#include <iostream>

#include "ALGORITHM/pid.hpp"
#include "mock_plant.hpp"
#include "test_helper.hpp"

namespace robot_foc::test
{
    /**
     * @brief PiController私有两阶段更新接口的单元测试访问器。
     * @note 仅转发调用，不保存状态，不参与生产运行。
     */
    class PiControllerTestAccess final
    {
    public:
        PiControllerTestAccess() = delete;

        [[nodiscard]] static common::ErrorCode prepare_update(
            const algorithm::PiController& controller,
            const algorithm::PiPreview& preview,
            common::Volts limited_output,
            bool limit_flag,
            common::Volts& next_integral_out
        ) noexcept
        {
            return controller.prepare_update(
                preview,
                limited_output,
                limit_flag,
                next_integral_out
            );
        }

        static void commit(
            algorithm::PiController& controller,
            common::Volts next_integral
        ) noexcept
        {
            controller.commit(next_integral);
        }
    };
} // namespace robot_foc::test

namespace
{
    using robot_foc::algorithm::PiConfig;
    using robot_foc::algorithm::PiController;
    using robot_foc::algorithm::PiPreview;
    using robot_foc::common::Amperes;
    using robot_foc::common::ErrorCode;
    using robot_foc::common::Seconds;
    using robot_foc::common::Volts;
    using robot_foc::test::PiControllerTestAccess;

    [[nodiscard]] PiConfig make_config(
        float kp,
        float ki,
        Seconds sample_time,
        Volts integral_min,
        Volts integral_max
    ) noexcept
    {
        PiConfig config {};
        config.kp_ = kp;
        config.ki_ = ki;
        config.sample_time_ = sample_time;
        config.integral_min_ = integral_min;
        config.integral_max_ = integral_max;
        return config;
    }

    [[nodiscard]] ErrorCode prepare_and_commit(
        PiController& controller,
        const PiPreview& preview,
        Volts limited_output,
        bool limit_flag
    ) noexcept
    {
        Volts next_integral {0.0F};
        const ErrorCode result = PiControllerTestAccess::prepare_update(
            controller,
            preview,
            limited_output,
            limit_flag,
            next_integral
        );
        if (result == ErrorCode::Ok)
        {
            PiControllerTestAccess::commit(controller, next_integral);
        }
        return result;
    }
} // namespace

/**
 * @brief 验证PiController配置、两阶段更新、抗饱和和错误处理。
 */
void test_pi_controller()
{
    // 测试1：未配置保护，失败时输出参数保持不变。
    {
        PiController controller {};
        PiPreview preview {};
        preview.error_ = Amperes{7.0F};
        preview.request_output_ = Volts{8.0F};

        const ErrorCode preview_result = controller.preview(
            Amperes{1.0F},
            Amperes{0.0F},
            preview
        );
        ROBOT_FOC_TEST_EXPECT(preview_result == ErrorCode::ErrorNotReady, "pi preview before configure rejected");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.error_, 7.0F), "pi failed preview preserves error output");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.request_output_, 8.0F), "pi failed preview preserves voltage output");

        Volts next_integral {9.0F};
        const ErrorCode prepare_result = PiControllerTestAccess::prepare_update(
            controller,
            preview,
            Volts{0.0F},
            false,
            next_integral
        );
        ROBOT_FOC_TEST_EXPECT(prepare_result == ErrorCode::ErrorNotReady, "pi prepare before configure rejected");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(next_integral, 9.0F), "pi failed prepare preserves candidate output");
    }

    // 测试2：正常积分采用两阶段提交，prepare不会提前修改内部状态。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 100.0F, Seconds{0.001F}, Volts{-20.0F}, Volts{20.0F});
        ROBOT_FOC_TEST_EXPECT(controller.configure(config) == ErrorCode::Ok, "pi valid configure accepted");

        PiPreview preview {};
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{2.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi first preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.error_, 2.0F), "pi first preview error correct");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.request_output_, 2.0F), "pi first preview voltage correct");

        Volts next_integral {99.0F};
        ROBOT_FOC_TEST_EXPECT(
            PiControllerTestAccess::prepare_update(
                controller,
                preview,
                Volts{2.0F},
                false,
                next_integral
            ) == ErrorCode::Ok,
            "pi prepare normal update succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(next_integral, 0.2F), "pi prepared integral increment correct");

        PiPreview before_commit {};
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{2.0F}, Amperes{0.0F}, before_commit) == ErrorCode::Ok,
            "pi preview before commit succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(before_commit.request_output_, 2.0F), "pi prepare does not mutate state");

        PiControllerTestAccess::commit(controller, next_integral);
        PiPreview after_commit {};
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{2.0F}, Amperes{0.0F}, after_commit) == ErrorCode::Ok,
            "pi preview after commit succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(after_commit.request_output_, 2.2F), "pi commit activates prepared integral");
    }

    // 测试3：正向和负向饱和时，误差继续加剧饱和则冻结积分。
    {
        PiController positive {};
        PiController negative {};
        const PiConfig config = make_config(1.0F, 100.0F, Seconds{0.001F}, Volts{-20.0F}, Volts{20.0F});
        ROBOT_FOC_TEST_EXPECT(positive.configure(config) == ErrorCode::Ok, "pi positive anti-windup configured");
        ROBOT_FOC_TEST_EXPECT(negative.configure(config) == ErrorCode::Ok, "pi negative anti-windup configured");

        PiPreview positive_preview {};
        ROBOT_FOC_TEST_EXPECT(
            positive.preview(Amperes{5.0F}, Amperes{0.0F}, positive_preview) == ErrorCode::Ok,
            "pi positive anti-windup preview succeeded"
        );
        Volts positive_next {99.0F};
        ROBOT_FOC_TEST_EXPECT(
            PiControllerTestAccess::prepare_update(
                positive,
                positive_preview,
                Volts{3.0F},
                true,
                positive_next
            ) == ErrorCode::Ok,
            "pi positive anti-windup prepare succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(positive_next, 0.0F), "pi positive saturation freezes integral");

        PiPreview negative_preview {};
        ROBOT_FOC_TEST_EXPECT(
            negative.preview(Amperes{-5.0F}, Amperes{0.0F}, negative_preview) == ErrorCode::Ok,
            "pi negative anti-windup preview succeeded"
        );
        Volts negative_next {99.0F};
        ROBOT_FOC_TEST_EXPECT(
            PiControllerTestAccess::prepare_update(
                negative,
                negative_preview,
                Volts{-3.0F},
                true,
                negative_next
            ) == ErrorCode::Ok,
            "pi negative anti-windup prepare succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(negative_next, 0.0F), "pi negative saturation freezes integral");
    }

    // 测试4：饱和时误差反向允许积分退出饱和。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 1.0F, Seconds{1.0F}, Volts{-10.0F}, Volts{10.0F});
        ROBOT_FOC_TEST_EXPECT(controller.configure(config) == ErrorCode::Ok, "pi recovery configure succeeded");

        PiPreview preview {};
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{1.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi recovery seed preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            prepare_and_commit(controller, preview, Volts{1.0F}, false) == ErrorCode::Ok,
            "pi recovery seed state committed"
        );

        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{-0.5F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi recovery reverse preview succeeded"
        );
        Volts next_integral {99.0F};
        ROBOT_FOC_TEST_EXPECT(
            PiControllerTestAccess::prepare_update(
                controller,
                preview,
                Volts{0.2F},
                true,
                next_integral
            ) == ErrorCode::Ok,
            "pi reverse error prepare succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(next_integral, 0.5F), "pi reverse error reduces saturated integral");
    }

    // 测试5：积分输出在正负上下限处钳位。
    {
        PiController controller {};
        const PiConfig config = make_config(0.0F, 10000.0F, Seconds{0.001F}, Volts{-5.0F}, Volts{5.0F});
        ROBOT_FOC_TEST_EXPECT(controller.configure(config) == ErrorCode::Ok, "pi clamp configure succeeded");

        PiPreview preview {};
        for (int index = 0; index < 3; ++index)
        {
            ROBOT_FOC_TEST_EXPECT(
                controller.preview(Amperes{1.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
                "pi upper clamp preview succeeded"
            );
            ROBOT_FOC_TEST_EXPECT(
                prepare_and_commit(controller, preview, Volts{0.0F}, false) == ErrorCode::Ok,
                "pi upper clamp update succeeded"
            );
        }
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{0.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi upper clamp verification preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.request_output_, 5.0F), "pi integral upper limit enforced");

        controller.reset();
        for (int index = 0; index < 3; ++index)
        {
            ROBOT_FOC_TEST_EXPECT(
                controller.preview(Amperes{-1.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
                "pi lower clamp preview succeeded"
            );
            ROBOT_FOC_TEST_EXPECT(
                prepare_and_commit(controller, preview, Volts{0.0F}, false) == ErrorCode::Ok,
                "pi lower clamp update succeeded"
            );
        }
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{0.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi lower clamp verification preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.request_output_, -5.0F), "pi integral lower limit enforced");
    }

    // 测试6：非法输入被拒绝，候选输出和preview输出保持不变。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 100.0F, Seconds{0.001F}, Volts{-10.0F}, Volts{10.0F});
        ROBOT_FOC_TEST_EXPECT(controller.configure(config) == ErrorCode::Ok, "pi invalid-input configure succeeded");

        PiPreview invalid_preview {};
        invalid_preview.error_ = Amperes{NAN};
        invalid_preview.request_output_ = Volts{5.0F};
        Volts next_integral {77.0F};
        const ErrorCode prepare_result = PiControllerTestAccess::prepare_update(
            controller,
            invalid_preview,
            Volts{3.0F},
            true,
            next_integral
        );
        ROBOT_FOC_TEST_EXPECT(prepare_result == ErrorCode::ErrorInvalidParam, "pi prepare rejects nan preview");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(next_integral, 77.0F), "pi failed prepare preserves candidate output");

        PiPreview preview_output {};
        preview_output.error_ = Amperes{11.0F};
        preview_output.request_output_ = Volts{12.0F};
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{INFINITY}, Amperes{0.0F}, preview_output)
                == ErrorCode::ErrorInvalidParam,
            "pi preview rejects infinite setpoint"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview_output.error_, 11.0F), "pi invalid preview preserves error output");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview_output.request_output_, 12.0F), "pi invalid preview preserves voltage output");
    }

    // 测试7：非法重新配置不会覆盖旧配置和积分状态。
    {
        PiController controller {};
        const PiConfig valid_config = make_config(2.0F, 80.0F, Seconds{0.001F}, Volts{-12.0F}, Volts{12.0F});
        ROBOT_FOC_TEST_EXPECT(controller.configure(valid_config) == ErrorCode::Ok, "pi initial configure succeeded");

        PiPreview preview {};
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{2.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi state seed preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            prepare_and_commit(controller, preview, Volts{4.0F}, false) == ErrorCode::Ok,
            "pi state seed update succeeded"
        );

        PiConfig invalid_config = valid_config;
        invalid_config.kp_ = -1.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid_config) == ErrorCode::ErrorInvalidParam,
            "pi invalid reconfigure rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{0.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi old configuration remains usable"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.request_output_, 0.16F), "pi invalid configure preserves integral state");
    }

    // 测试8：reset只清除积分状态，不清除配置。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 10.0F, Seconds{0.1F}, Volts{-5.0F}, Volts{5.0F});
        ROBOT_FOC_TEST_EXPECT(controller.configure(config) == ErrorCode::Ok, "pi reset configure succeeded");

        PiPreview preview {};
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{1.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi reset seed preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            prepare_and_commit(controller, preview, Volts{1.0F}, false) == ErrorCode::Ok,
            "pi reset seed update succeeded"
        );

        controller.reset();
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(Amperes{0.0F}, Amperes{0.0F}, preview) == ErrorCode::Ok,
            "pi remains configured after reset"
        );
        ROBOT_FOC_TEST_EXPECT(nearly_equal(preview.request_output_, 0.0F), "pi reset clears integral state");
    }
}

/**
 * @brief 使用一阶被控对象验证PiController闭环阶跃跟踪和超调。
 */
void test_pi_controller_closed()
{
    using robot_foc::algorithm::PiConfig;
    using robot_foc::algorithm::PiController;
    using robot_foc::algorithm::PiPreview;
    using robot_foc::common::Amperes;
    using robot_foc::common::ErrorCode;
    using robot_foc::common::Seconds;
    using robot_foc::common::Volts;
    using robot_foc::test::mock::MockFirstOrderPlant;
    using robot_foc::test::PiControllerTestAccess;

    constexpr Seconds kSampleTime {0.01F};
    constexpr float kPlantTimeConstant {0.2F};
    constexpr Amperes kSetpoint {1.0F};
    constexpr Volts kVoltageLimit {1.0F};
    constexpr std::size_t kSimulationSteps {300U};
    constexpr float kSteadyStateTolerance {0.1F};
    constexpr float kMaximumOvershootRatio {0.15F};

    PiController controller {};
    PiConfig config {};
    config.kp_ = 1.2F;
    config.ki_ = 2.5F;
    config.sample_time_ = kSampleTime;
    config.integral_min_ = Volts{-5.0F};
    config.integral_max_ = Volts{5.0F};
    ROBOT_FOC_TEST_EXPECT(controller.configure(config) == ErrorCode::Ok, "pi closed-loop configure succeeded");

    MockFirstOrderPlant plant(kSampleTime, kPlantTimeConstant);
    plant.reset();

    Amperes feedback {0.0F};
    Amperes peak_feedback {0.0F};
    for (std::size_t step = 0U; step < kSimulationSteps; ++step)
    {
        PiPreview preview {};
        const ErrorCode preview_result = controller.preview(kSetpoint, feedback, preview);
        ROBOT_FOC_TEST_EXPECT(preview_result == ErrorCode::Ok, "pi closed-loop preview succeeded");

        Volts limited_output = preview.request_output_;
        bool limit_flag {false};
        if (limited_output > kVoltageLimit)
        {
            limited_output = kVoltageLimit;
            limit_flag = true;
        }
        else if (limited_output < -kVoltageLimit)
        {
            limited_output = -kVoltageLimit;
            limit_flag = true;
        }

        Volts next_integral {0.0F};
        const ErrorCode prepare_result = PiControllerTestAccess::prepare_update(
            controller,
            preview,
            limited_output,
            limit_flag,
            next_integral
        );
        ROBOT_FOC_TEST_EXPECT(prepare_result == ErrorCode::Ok, "pi closed-loop prepare succeeded");
        if (prepare_result == ErrorCode::Ok)
        {
            PiControllerTestAccess::commit(controller, next_integral);
        }

        feedback = plant.update(limited_output);
        if (feedback > peak_feedback)
        {
            peak_feedback = feedback;
        }
    }

    ROBOT_FOC_TEST_EXPECT(
        std::fabs(feedback - kSetpoint) < kSteadyStateTolerance,
        "pi closed-loop steady-state error within tolerance"
    );
    const float maximum_peak = kSetpoint * (1.0F + kMaximumOvershootRatio);
    ROBOT_FOC_TEST_EXPECT(
        peak_feedback <= maximum_peak,
        "pi closed-loop overshoot within tolerance"
    );
    /*
    std::cout << "\n[PiController Closed Step] set=" << kSetpoint
              << " | peak_fb=" << peak_feedback
              << " | final_fb=" << feedback
              << " | allowed_peak=" << maximum_peak << '\n';
    */
}
