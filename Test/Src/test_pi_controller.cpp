/**
 * @file test_pi_controller.cpp
 * @brief 分步式PI控制器单元测试。
 * @details 验证配置校验、两阶段更新、抗积分饱和、状态版本校验、
 * 双控制器成对提交及闭环阶跃响应。
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "ALGORITHM/pid.hpp"
#include "mock_plant.hpp"
#include "test_helper.hpp"

namespace
{
    using robot_foc::algorithm::PiConfig;
    using robot_foc::algorithm::PiController;
    using robot_foc::common::ErrorCode;
    using PiPreview = PiController::PiPreview;
    using PreparedState = PiController::PreparedState;

    [[nodiscard]] PiConfig make_config(
        float kp,
        float ki,
        float sample_time,
        float integral_min,
        float integral_max
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
} // namespace

/**
 * @brief 验证PiController配置、分步更新、抗饱和和状态有效性。
 */
void test_pi_controller()
{
    // 测试1：未配置保护和配置参数校验。
    {
        PiController controller {};
        PiPreview preview {};
        PreparedState state {};

        ROBOT_FOC_TEST_EXPECT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::ErrorNotReady,
            "pi preview before configure rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.prepare_update(preview, 0.0F, false, state)
                == ErrorCode::ErrorNotReady,
            "pi prepare before configure rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.commit(state) == ErrorCode::ErrorNotReady,
            "pi commit before configure rejected"
        );

        const PiConfig valid = make_config(1.0F, 100.0F, 0.001F, -20.0F, 20.0F);
        PiConfig invalid = valid;
        invalid.kp_ = -1.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi negative kp rejected"
        );

        invalid = valid;
        invalid.ki_ = -1.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi negative ki rejected"
        );

        invalid = valid;
        invalid.sample_time_ = 0.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi zero sample time rejected"
        );

        invalid = valid;
        invalid.integral_min_ = 1.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi positive integral minimum rejected"
        );

        invalid = valid;
        invalid.integral_max_ = -1.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi negative integral maximum rejected"
        );

        invalid = valid;
        invalid.kp_ = std::numeric_limits<float>::quiet_NaN();
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi nonfinite configuration rejected"
        );

        invalid = valid;
        invalid.ki_ = std::numeric_limits<float>::max();
        invalid.sample_time_ = 2.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi overflowing discrete gain rejected"
        );

        ROBOT_FOC_TEST_EXPECT(
            controller.configure(valid) == ErrorCode::Ok,
            "pi valid configuration accepted"
        );
    }

    // 测试2：prepare_update不提前修改积分，commit后候选状态才生效。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 100.0F, 0.001F, -20.0F, 20.0F);
        ROBOT_FOC_TEST_ASSERT(
            controller.configure(config) == ErrorCode::Ok,
            "pi transaction configure succeeded"
        );

        PiPreview preview {};
        PreparedState state {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(2.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi transaction preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), 2.0F),
            "pi proportional output correct"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 2.0F, false, state) == ErrorCode::Ok,
            "pi transaction prepare succeeded"
        );

        PiPreview before_commit {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, before_commit) == ErrorCode::Ok,
            "pi preview before commit succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(before_commit.get_request_output(), 0.0F),
            "pi prepare does not change committed integral"
        );

        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi transaction commit succeeded"
        );

        PiPreview after_commit {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, after_commit) == ErrorCode::Ok,
            "pi preview after commit succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(after_commit.get_request_output(), 0.2F),
            "pi committed integral increment correct"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.commit(state) == ErrorCode::ErrorUnsupported,
            "pi repeated commit rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.prepare_update(preview, 2.0F, false, state)
                == ErrorCode::ErrorUnsupported,
            "pi old preview rejected after commit"
        );
    }

    // 测试3：同向饱和冻结积分，反向误差允许积分退出饱和。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 1.0F, 1.0F, -10.0F, 10.0F);
        ROBOT_FOC_TEST_ASSERT(
            controller.configure(config) == ErrorCode::Ok,
            "pi anti-windup configure succeeded"
        );

        PiPreview preview {};
        PreparedState state {};

        ROBOT_FOC_TEST_ASSERT(
            controller.preview(5.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi positive saturation preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 3.0F, true, state) == ErrorCode::Ok,
            "pi positive saturation prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi positive saturation commit succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi positive saturation verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), 0.0F),
            "pi positive saturation freezes integral"
        );

        ROBOT_FOC_TEST_ASSERT(
            controller.preview(-5.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi negative saturation preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, -3.0F, true, state) == ErrorCode::Ok,
            "pi negative saturation prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi negative saturation commit succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi negative saturation verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), 0.0F),
            "pi negative saturation freezes integral"
        );

        ROBOT_FOC_TEST_ASSERT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi recovery seed preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 1.0F, false, state) == ErrorCode::Ok,
            "pi recovery seed prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi recovery seed commit succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(-0.5F, 0.0F, preview) == ErrorCode::Ok,
            "pi reverse error preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 0.2F, true, state) == ErrorCode::Ok,
            "pi reverse error prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi reverse error commit succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi reverse error verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), 0.5F),
            "pi reverse error reduces saturated integral"
        );
    }

    // 测试4：积分输出在配置的正负上下限处钳位。
    {
        PiController controller {};
        const PiConfig config = make_config(0.0F, 10000.0F, 0.001F, -5.0F, 5.0F);
        ROBOT_FOC_TEST_ASSERT(
            controller.configure(config) == ErrorCode::Ok,
            "pi integral clamp configure succeeded"
        );

        PiPreview preview {};
        PreparedState state {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi upper clamp preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 0.0F, false, state) == ErrorCode::Ok,
            "pi upper clamp prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi upper clamp commit succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi upper clamp verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), 5.0F),
            "pi integral upper limit enforced"
        );

        controller.reset();
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(-1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi lower clamp preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 0.0F, false, state) == ErrorCode::Ok,
            "pi lower clamp prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi lower clamp commit succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi lower clamp verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), -5.0F),
            "pi integral lower limit enforced"
        );
    }

    // 测试5：非法输入使复用的输出对象失效，且不污染已提交状态。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 100.0F, 0.001F, -10.0F, 10.0F);
        ROBOT_FOC_TEST_ASSERT(
            controller.configure(config) == ErrorCode::Ok,
            "pi invalid-input configure succeeded"
        );

        PiPreview preview {};
        PreparedState state {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi valid preview before invalid input succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.preview(
                std::numeric_limits<float>::quiet_NaN(),
                0.0F,
                preview
            ) == ErrorCode::ErrorInvalidParam,
            "pi nonfinite setpoint rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.prepare_update(preview, 0.0F, false, state)
                == ErrorCode::ErrorUnsupported,
            "pi failed preview invalidates reused preview"
        );

        ROBOT_FOC_TEST_ASSERT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi valid preview before invalid limit succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 1.0F, false, state) == ErrorCode::Ok,
            "pi valid candidate before invalid limit succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.prepare_update(
                preview,
                std::numeric_limits<float>::infinity(),
                false,
                state
            ) == ErrorCode::ErrorInvalidParam,
            "pi nonfinite limited output rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.commit(state) == ErrorCode::ErrorUnsupported,
            "pi failed prepare invalidates reused candidate"
        );

        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi invalid-input state verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), 0.0F),
            "pi invalid input preserves committed integral"
        );
    }

    // 测试6：Ki_step与误差相乘溢出时拒绝候选状态，积分保持不变。
    {
        PiController controller {};
        const float maximum = std::numeric_limits<float>::max();
        const PiConfig config = make_config(
            0.0F,
            maximum,
            1.0F,
            -maximum,
            maximum
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.configure(config) == ErrorCode::Ok,
            "pi overflow configure succeeded"
        );

        PiPreview preview {};
        PreparedState state {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(2.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi overflow preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.prepare_update(preview, 0.0F, false, state)
                == ErrorCode::ErrorInvalidParam,
            "pi ki step times error overflow rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.commit(state) == ErrorCode::ErrorUnsupported,
            "pi overflow invalidates candidate"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(0.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi overflow state verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(preview.get_request_output(), 0.0F),
            "pi overflow preserves committed integral"
        );
    }

    // 测试7：非法配置保留旧状态；reset和成功配置使旧状态失效。
    {
        PiController controller {};
        const PiConfig config = make_config(1.0F, 100.0F, 0.001F, -10.0F, 10.0F);
        ROBOT_FOC_TEST_ASSERT(
            controller.configure(config) == ErrorCode::Ok,
            "pi revision configure succeeded"
        );

        PiPreview preview {};
        PreparedState state {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi pending candidate preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 1.0F, false, state) == ErrorCode::Ok,
            "pi pending candidate prepare succeeded"
        );

        PiConfig invalid = config;
        invalid.kp_ = -1.0F;
        ROBOT_FOC_TEST_EXPECT(
            controller.configure(invalid) == ErrorCode::ErrorInvalidParam,
            "pi invalid reconfigure rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.commit(state) == ErrorCode::Ok,
            "pi invalid reconfigure preserves pending candidate"
        );

        ROBOT_FOC_TEST_ASSERT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi preview before reset succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 1.1F, false, state) == ErrorCode::Ok,
            "pi candidate before reset succeeded"
        );
        controller.reset();
        ROBOT_FOC_TEST_EXPECT(
            controller.commit(state) == ErrorCode::ErrorUnsupported,
            "pi reset invalidates old candidate"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.prepare_update(preview, 1.1F, false, state)
                == ErrorCode::ErrorUnsupported,
            "pi reset invalidates old preview"
        );

        ROBOT_FOC_TEST_ASSERT(
            controller.preview(1.0F, 0.0F, preview) == ErrorCode::Ok,
            "pi preview before reconfigure succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(preview, 1.0F, false, state) == ErrorCode::Ok,
            "pi candidate before reconfigure succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.configure(config) == ErrorCode::Ok,
            "pi valid reconfigure succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.commit(state) == ErrorCode::ErrorUnsupported,
            "pi valid reconfigure invalidates old candidate"
        );
        ROBOT_FOC_TEST_EXPECT(
            controller.prepare_update(preview, 1.0F, false, state)
                == ErrorCode::ErrorUnsupported,
            "pi valid reconfigure invalidates old preview"
        );
    }

    // 测试8：来源检查和commit_pair保证两路状态全部有效后再统一提交。
    {
        PiController first {};
        PiController second {};
        const PiConfig config = make_config(1.0F, 100.0F, 0.001F, -10.0F, 10.0F);
        ROBOT_FOC_TEST_ASSERT(
            first.configure(config) == ErrorCode::Ok,
            "pi pair first configure succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            second.configure(config) == ErrorCode::Ok,
            "pi pair second configure succeeded"
        );

        PiPreview first_preview {};
        PiPreview second_preview {};
        PreparedState first_state {};
        PreparedState second_state {};
        ROBOT_FOC_TEST_ASSERT(
            first.preview(1.0F, 0.0F, first_preview) == ErrorCode::Ok,
            "pi pair first preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            second.preview(2.0F, 0.0F, second_preview) == ErrorCode::Ok,
            "pi pair second preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            second.prepare_update(first_preview, 1.0F, false, second_state)
                == ErrorCode::ErrorUnsupported,
            "pi foreign preview rejected"
        );
        ROBOT_FOC_TEST_ASSERT(
            first.prepare_update(first_preview, 1.0F, false, first_state)
                == ErrorCode::Ok,
            "pi pair first prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            second.prepare_update(second_preview, 2.0F, false, second_state)
                == ErrorCode::Ok,
            "pi pair second prepare succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            second.commit(first_state) == ErrorCode::ErrorUnsupported,
            "pi foreign candidate rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            PiController::commit_pair(first, first_state, first, first_state)
                == ErrorCode::ErrorInvalidParam,
            "pi pair rejects identical controller instances"
        );

        first.reset();
        ROBOT_FOC_TEST_EXPECT(
            PiController::commit_pair(first, first_state, second, second_state)
                == ErrorCode::ErrorUnsupported,
            "pi stale first candidate rejects whole pair"
        );

        PiPreview verification {};
        ROBOT_FOC_TEST_ASSERT(
            first.preview(0.0F, 0.0F, verification) == ErrorCode::Ok,
            "pi first-axis failure first verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(verification.get_request_output(), 0.0F),
            "pi first-axis failure leaves first controller unchanged"
        );
        ROBOT_FOC_TEST_ASSERT(
            second.preview(0.0F, 0.0F, verification) == ErrorCode::Ok,
            "pi first-axis failure second verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(verification.get_request_output(), 0.0F),
            "pi first-axis failure leaves second controller unchanged"
        );

        ROBOT_FOC_TEST_ASSERT(
            first.preview(1.0F, 0.0F, first_preview) == ErrorCode::Ok,
            "pi pair refreshed first preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            first.prepare_update(first_preview, 1.0F, false, first_state)
                == ErrorCode::Ok,
            "pi pair refreshed first candidate succeeded"
        );

        second.reset();
        ROBOT_FOC_TEST_EXPECT(
            PiController::commit_pair(first, first_state, second, second_state)
                == ErrorCode::ErrorUnsupported,
            "pi stale second candidate rejects whole pair"
        );

        ROBOT_FOC_TEST_ASSERT(
            first.preview(0.0F, 0.0F, verification) == ErrorCode::Ok,
            "pi second-axis failure first verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(verification.get_request_output(), 0.0F),
            "pi second-axis failure leaves first controller unchanged"
        );
        ROBOT_FOC_TEST_ASSERT(
            second.preview(0.0F, 0.0F, verification) == ErrorCode::Ok,
            "pi second-axis failure second verification succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(verification.get_request_output(), 0.0F),
            "pi second-axis failure leaves second controller unchanged"
        );

        ROBOT_FOC_TEST_ASSERT(
            second.preview(2.0F, 0.0F, second_preview) == ErrorCode::Ok,
            "pi pair refreshed second preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            second.prepare_update(second_preview, 2.0F, false, second_state)
                == ErrorCode::Ok,
            "pi pair refreshed second candidate succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            PiController::commit_pair(first, first_state, second, second_state)
                == ErrorCode::Ok,
            "pi pair commit succeeded"
        );

        ROBOT_FOC_TEST_ASSERT(
            first.preview(0.0F, 0.0F, first_preview) == ErrorCode::Ok,
            "pi pair first result preview succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            second.preview(0.0F, 0.0F, second_preview) == ErrorCode::Ok,
            "pi pair second result preview succeeded"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(first_preview.get_request_output(), 0.1F),
            "pi pair first integral committed"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(second_preview.get_request_output(), 0.2F),
            "pi pair second integral committed"
        );
    }
}

/**
 * @brief 使用一阶被控对象验证PiController闭环阶跃跟踪和超调。
 */
void test_pi_controller_closed()
{
    using robot_foc::test::mock::MockFirstOrderPlant;

    constexpr float kSampleTime {0.01F};
    constexpr float kPlantTimeConstant {0.2F};
    constexpr float kSetpoint {1.0F};
    constexpr float kOutputMinimum {-1.0F};
    constexpr float kOutputMaximum {1.0F};
    constexpr std::size_t kSimulationSteps {300U};
    constexpr float kSteadyStateTolerance {0.1F};
    constexpr float kMaximumOvershootRatio {0.15F};

    PiController controller {};
    const PiConfig config = make_config(
        1.2F,
        2.5F,
        kSampleTime,
        -5.0F,
        5.0F
    );
    ROBOT_FOC_TEST_ASSERT(
        controller.configure(config) == ErrorCode::Ok,
        "pi closed-loop configure succeeded"
    );

    MockFirstOrderPlant plant {kSampleTime, kPlantTimeConstant};
    plant.reset();

    float feedback {0.0F};
    float peak_feedback {0.0F};
    for (std::size_t step = 0U; step < kSimulationSteps; ++step)
    {
        PiPreview preview {};
        PreparedState state {};
        ROBOT_FOC_TEST_ASSERT(
            controller.preview(kSetpoint, feedback, preview) == ErrorCode::Ok,
            "pi closed-loop preview succeeded"
        );

        const float requested_output = preview.get_request_output();
        const float limited_output = std::clamp(
            requested_output,
            kOutputMinimum,
            kOutputMaximum
        );
        const bool limit_flag = !nearly_equal(limited_output, requested_output);

        ROBOT_FOC_TEST_ASSERT(
            controller.prepare_update(
                preview,
                limited_output,
                limit_flag,
                state
            ) == ErrorCode::Ok,
            "pi closed-loop prepare succeeded"
        );
        ROBOT_FOC_TEST_ASSERT(
            controller.commit(state) == ErrorCode::Ok,
            "pi closed-loop commit succeeded"
        );

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
}
