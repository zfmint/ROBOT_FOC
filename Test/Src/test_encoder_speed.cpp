/**
 * @file test_encoder_speed.cpp
 * @brief EncoderSpeedEstimator PC端单元测试
 */

#include <cmath>
#include <cstdint>
#include <limits>

#include "ALGORITHM/encoder_speed.hpp"
#include "error_code.hpp"
#include "mock_encoder_counter.hpp"
#include "mock_plant.hpp"
#include "test_helper.hpp"

void test_encoder_speed()
{
    using robot_foc::algorithm::EncoderSpeedConfig;
    using robot_foc::algorithm::EncoderSpeedEstimator;
    using robot_foc::algorithm::EncoderSpeedSample;
    using robot_foc::common::ErrorCode;

    // 未配置时拒绝更新，输出保持不变。
    {
        EncoderSpeedEstimator estimator{};
        EncoderSpeedSample sample{123, 12.0F, 34.0F};
        const ErrorCode result = estimator.update(100U, 0.01F, sample);

        ROBOT_FOC_TEST_EXPECT(result == ErrorCode::ErrorNotReady,"encoder update before configure rejected");
        ROBOT_FOC_TEST_EXPECT(sample.delta_counts == 123,"unconfigured update keeps output");
        ROBOT_FOC_TEST_EXPECT(!estimator.is_configured(),"encoder is not configured by default");
    }

    // 配置参数校验。
    {
        EncoderSpeedEstimator estimator{};

        const ErrorCode zero_ppr = estimator.configure(EncoderSpeedConfig{0U, 16U, 4U});
        const ErrorCode invalid_bits = estimator.configure(EncoderSpeedConfig{100U, 7U, 4U});
        const ErrorCode invalid_gain = estimator.configure(EncoderSpeedConfig{100U, 16U, 3U});
        const std::uint32_t overflow_ppr =
            (std::numeric_limits<std::uint32_t>::max() / 4U) + 1U;
        const ErrorCode overflow = estimator.configure(
            EncoderSpeedConfig{overflow_ppr, 16U, 4U});

        ROBOT_FOC_TEST_EXPECT(zero_ppr == ErrorCode::ErrorInvalidParam,"zero encoder PPR rejected");
        ROBOT_FOC_TEST_EXPECT(invalid_bits == ErrorCode::ErrorInvalidParam,"unsupported counter width rejected");
        ROBOT_FOC_TEST_EXPECT(invalid_gain == ErrorCode::ErrorInvalidParam,"unsupported quadrature gain rejected");
        ROBOT_FOC_TEST_EXPECT(overflow == ErrorCode::ErrorInvalidParam,"PPR times gain overflow rejected");
    }

    // 倍频系数来自配置，内部完成PPR到CPR换算。
    {
        EncoderSpeedEstimator estimator{};

        const ErrorCode gain_1 = estimator.configure(EncoderSpeedConfig{100U, 16U, 1U});
        ROBOT_FOC_TEST_EXPECT(gain_1 == ErrorCode::Ok,"quadrature gain 1 accepted");
        ROBOT_FOC_TEST_EXPECT(estimator.counts_per_revolution() == 100U,"gain 1 produces correct CPR");

        const ErrorCode gain_2 = estimator.configure(EncoderSpeedConfig{100U, 16U, 2U});
        ROBOT_FOC_TEST_EXPECT(gain_2 == ErrorCode::Ok,"quadrature gain 2 accepted");
        ROBOT_FOC_TEST_EXPECT(estimator.counts_per_revolution() == 200U,"gain 2 produces correct CPR");

        const ErrorCode gain_4 = estimator.configure(EncoderSpeedConfig{100U, 16U, 4U});
        ROBOT_FOC_TEST_EXPECT(gain_4 == ErrorCode::Ok,"quadrature gain 4 accepted");
        ROBOT_FOC_TEST_EXPECT(estimator.counts_per_revolution() == 400U,"gain 4 produces correct CPR");
    }

    // 首次更新只建立基线并输出零速。
    {
        EncoderSpeedEstimator estimator{};
        EncoderSpeedSample sample{7, 8.0F, 9.0F};
        const ErrorCode configured = estimator.configure(EncoderSpeedConfig{100U, 16U, 4U});
        const ErrorCode updated = estimator.update(1000U, 0.1F, sample);

        ROBOT_FOC_TEST_EXPECT(configured == ErrorCode::Ok,"baseline case configured");
        ROBOT_FOC_TEST_EXPECT(updated == ErrorCode::Ok,"first encoder sample accepted");
        ROBOT_FOC_TEST_EXPECT(estimator.has_baseline(),"first sample establishes baseline");
        ROBOT_FOC_TEST_EXPECT(sample.delta_counts == 0,"first sample delta is zero");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(sample.raw_rpm, 0.0F),"first sample raw speed is zero");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(sample.filtered_rpm, 0.0F),"first sample filtered speed is zero");
    }

    // 正向和反向RPM计算：PPR=100，4倍频后CPR=400，40计数/0.1秒等于60RPM。
    {
        EncoderSpeedEstimator forward{};
        EncoderSpeedSample forward_sample{};
        const ErrorCode forward_config = forward.configure(EncoderSpeedConfig{100U, 16U, 4U});
        const ErrorCode forward_baseline = forward.update(1000U, 0.1F, forward_sample);
        const ErrorCode forward_update = forward.update(1040U, 0.1F, forward_sample);

        ROBOT_FOC_TEST_EXPECT(forward_config == ErrorCode::Ok,"forward speed configured");
        ROBOT_FOC_TEST_EXPECT(forward_baseline == ErrorCode::Ok,"forward baseline accepted");
        ROBOT_FOC_TEST_EXPECT(forward_update == ErrorCode::Ok,"forward speed update accepted");
        ROBOT_FOC_TEST_EXPECT(forward_sample.delta_counts == 40,"forward delta is positive");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(forward_sample.raw_rpm, 60.0F),"forward RPM calculation correct");

        EncoderSpeedEstimator reverse{};
        EncoderSpeedSample reverse_sample{};
        const ErrorCode reverse_config = reverse.configure(EncoderSpeedConfig{100U, 16U, 4U});
        const ErrorCode reverse_baseline = reverse.update(1000U, 0.1F, reverse_sample);
        const ErrorCode reverse_update = reverse.update(960U, 0.1F, reverse_sample);

        ROBOT_FOC_TEST_EXPECT(reverse_config == ErrorCode::Ok,"reverse speed configured");
        ROBOT_FOC_TEST_EXPECT(reverse_baseline == ErrorCode::Ok,"reverse baseline accepted");
        ROBOT_FOC_TEST_EXPECT(reverse_update == ErrorCode::Ok,"reverse speed update accepted");
        ROBOT_FOC_TEST_EXPECT(reverse_sample.delta_counts == -40,"reverse delta is negative");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(reverse_sample.raw_rpm, -60.0F),"reverse RPM calculation correct");
    }

    // 16位计数器正向与反向回绕。
    {
        EncoderSpeedEstimator forward{};
        EncoderSpeedSample forward_sample{};
        const ErrorCode forward_config = forward.configure(EncoderSpeedConfig{100U, 16U, 4U});
        const ErrorCode forward_baseline = forward.update(65530U, 0.1F, forward_sample);
        const ErrorCode forward_update = forward.update(10U, 0.1F, forward_sample);

        ROBOT_FOC_TEST_EXPECT(forward_config == ErrorCode::Ok,"16-bit forward wrap configured");
        ROBOT_FOC_TEST_EXPECT(forward_baseline == ErrorCode::Ok,"16-bit forward wrap baseline accepted");
        ROBOT_FOC_TEST_EXPECT(forward_update == ErrorCode::Ok,"16-bit forward wrap accepted");
        ROBOT_FOC_TEST_EXPECT(forward_sample.delta_counts == 16,"16-bit forward wrap delta correct");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(forward_sample.raw_rpm, 24.0F),"16-bit forward wrap RPM correct");

        EncoderSpeedEstimator reverse{};
        EncoderSpeedSample reverse_sample{};
        const ErrorCode reverse_config = reverse.configure(EncoderSpeedConfig{100U, 16U, 4U});
        const ErrorCode reverse_baseline = reverse.update(5U, 0.1F, reverse_sample);
        const ErrorCode reverse_update = reverse.update(65525U, 0.1F, reverse_sample);

        ROBOT_FOC_TEST_EXPECT(reverse_config == ErrorCode::Ok,"16-bit reverse wrap configured");
        ROBOT_FOC_TEST_EXPECT(reverse_baseline == ErrorCode::Ok,"16-bit reverse wrap baseline accepted");
        ROBOT_FOC_TEST_EXPECT(reverse_update == ErrorCode::Ok,"16-bit reverse wrap accepted");
        ROBOT_FOC_TEST_EXPECT(reverse_sample.delta_counts == -16,"16-bit reverse wrap delta correct");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(reverse_sample.raw_rpm, -24.0F),"16-bit reverse wrap RPM correct");
    }

    // 32位计数器正向与反向回绕。
    {
        constexpr std::uint32_t kMaxCount = std::numeric_limits<std::uint32_t>::max();

        EncoderSpeedEstimator forward{};
        EncoderSpeedSample forward_sample{};
        const ErrorCode forward_config = forward.configure(EncoderSpeedConfig{100U, 32U, 4U});
        const ErrorCode forward_baseline = forward.update(kMaxCount - 5U, 0.1F, forward_sample);
        const ErrorCode forward_update = forward.update(10U, 0.1F, forward_sample);

        ROBOT_FOC_TEST_EXPECT(forward_config == ErrorCode::Ok,"32-bit forward wrap configured");
        ROBOT_FOC_TEST_EXPECT(forward_baseline == ErrorCode::Ok,"32-bit forward wrap baseline accepted");
        ROBOT_FOC_TEST_EXPECT(forward_update == ErrorCode::Ok,"32-bit forward wrap accepted");
        ROBOT_FOC_TEST_EXPECT(forward_sample.delta_counts == 16,"32-bit forward wrap delta correct");

        EncoderSpeedEstimator reverse{};
        EncoderSpeedSample reverse_sample{};
        const ErrorCode reverse_config = reverse.configure(EncoderSpeedConfig{100U, 32U, 4U});
        const ErrorCode reverse_baseline = reverse.update(5U, 0.1F, reverse_sample);
        const ErrorCode reverse_update = reverse.update(kMaxCount - 10U, 0.1F, reverse_sample);

        ROBOT_FOC_TEST_EXPECT(reverse_config == ErrorCode::Ok,"32-bit reverse wrap configured");
        ROBOT_FOC_TEST_EXPECT(reverse_baseline == ErrorCode::Ok,"32-bit reverse wrap baseline accepted");
        ROBOT_FOC_TEST_EXPECT(reverse_update == ErrorCode::Ok,"32-bit reverse wrap accepted");
        ROBOT_FOC_TEST_EXPECT(reverse_sample.delta_counts == -16,"32-bit reverse wrap delta correct");
    }

    // 非法计数值、半量程歧义和非法采样周期不能污染状态或输出。
    {
        EncoderSpeedEstimator estimator{};
        EncoderSpeedSample sample{123, 12.0F, 34.0F};
        const ErrorCode configured = estimator.configure(EncoderSpeedConfig{100U, 16U, 4U});
        const ErrorCode out_of_range = estimator.update(65536U, 0.1F, sample);

        ROBOT_FOC_TEST_EXPECT(configured == ErrorCode::Ok,"invalid-input case configured");
        ROBOT_FOC_TEST_EXPECT(out_of_range == ErrorCode::ErrorInvalidParam,"16-bit out-of-range count rejected");
        ROBOT_FOC_TEST_EXPECT(!estimator.has_baseline(),"invalid count does not establish baseline");
        ROBOT_FOC_TEST_EXPECT(sample.delta_counts == 123,"invalid count keeps output");

        const ErrorCode zero_period = estimator.update(0U, 0.0F, sample);
        const ErrorCode negative_period = estimator.update(0U, -0.1F, sample);
        const ErrorCode nan_period = estimator.update(
            0U, std::numeric_limits<float>::quiet_NaN(), sample);
        const ErrorCode infinite_period = estimator.update(
            0U, std::numeric_limits<float>::infinity(), sample);

        ROBOT_FOC_TEST_EXPECT(zero_period == ErrorCode::ErrorInvalidParam,"zero sample period rejected");
        ROBOT_FOC_TEST_EXPECT(negative_period == ErrorCode::ErrorInvalidParam,"negative sample period rejected");
        ROBOT_FOC_TEST_EXPECT(nan_period == ErrorCode::ErrorInvalidParam,"NaN sample period rejected");
        ROBOT_FOC_TEST_EXPECT(infinite_period == ErrorCode::ErrorInvalidParam,"infinite sample period rejected");
        ROBOT_FOC_TEST_EXPECT(!estimator.has_baseline(),"invalid periods do not establish baseline");

        const ErrorCode baseline = estimator.update(0U, 0.1F, sample);
        sample = EncoderSpeedSample{321, 21.0F, 43.0F};
        const ErrorCode ambiguous = estimator.update(32768U, 0.1F, sample);
        const ErrorCode recovery = estimator.update(1U, 0.1F, sample);

        ROBOT_FOC_TEST_EXPECT(baseline == ErrorCode::Ok,"ambiguity case baseline accepted");
        ROBOT_FOC_TEST_EXPECT(ambiguous == ErrorCode::ErrorInvalidParam,"half-range direction ambiguity rejected");
        ROBOT_FOC_TEST_EXPECT(recovery == ErrorCode::Ok,"valid sample accepted after ambiguous sample");
        ROBOT_FOC_TEST_EXPECT(sample.delta_counts == 1,"ambiguous sample does not advance baseline");
    }

    // 固定窗口为4：输入RPM序列1、2、3、4、5，滤波输出最终应为3.5。
    {
        EncoderSpeedEstimator estimator{};
        EncoderSpeedSample sample{};
        const ErrorCode configured = estimator.configure(EncoderSpeedConfig{60U, 16U, 1U});
        const ErrorCode baseline = estimator.update(100U, 1.0F, sample);

        bool all_updates_ok = (configured == ErrorCode::Ok) && (baseline == ErrorCode::Ok);
        const std::uint32_t counts[] = {101U, 103U, 106U, 110U, 115U};
        const float expected_raw[] = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F};
        const float expected_filtered[] = {1.0F, 1.5F, 2.0F, 2.5F, 3.5F};

        for (std::size_t index = 0U; index < 5U; ++index)
        {
            const ErrorCode result = estimator.update(counts[index], 1.0F, sample);
            all_updates_ok = all_updates_ok && (result == ErrorCode::Ok);
            ROBOT_FOC_TEST_EXPECT(nearly_equal(sample.raw_rpm, expected_raw[index]),"encoder raw speed sequence correct");
            ROBOT_FOC_TEST_EXPECT(nearly_equal(sample.filtered_rpm, expected_filtered[index]),"encoder moving average sequence correct");
        }

        ROBOT_FOC_TEST_EXPECT(all_updates_ok,"encoder filter sequence updates succeed");
    }

    // reset与成功重配置均清除基线和滤波历史，但保留有效配置。
    {
        EncoderSpeedEstimator estimator{};
        EncoderSpeedSample sample{};
        const ErrorCode first_config = estimator.configure(EncoderSpeedConfig{60U, 16U, 1U});
        const ErrorCode first_baseline = estimator.update(100U, 1.0F, sample);
        const ErrorCode first_speed = estimator.update(110U, 1.0F, sample);

        estimator.reset();
        const bool configured_after_reset = estimator.is_configured();
        const bool baseline_after_reset = estimator.has_baseline();
        const ErrorCode reset_baseline = estimator.update(200U, 1.0F, sample);
        const ErrorCode reset_speed = estimator.update(201U, 1.0F, sample);

        ROBOT_FOC_TEST_EXPECT(first_config == ErrorCode::Ok,"reset case configured");
        ROBOT_FOC_TEST_EXPECT(first_baseline == ErrorCode::Ok,"reset case baseline accepted");
        ROBOT_FOC_TEST_EXPECT(first_speed == ErrorCode::Ok,"reset case speed accepted");
        ROBOT_FOC_TEST_EXPECT(configured_after_reset,"reset preserves configuration");
        ROBOT_FOC_TEST_EXPECT(!baseline_after_reset,"reset clears baseline");
        ROBOT_FOC_TEST_EXPECT(reset_baseline == ErrorCode::Ok,"post-reset baseline accepted");
        ROBOT_FOC_TEST_EXPECT(reset_speed == ErrorCode::Ok,"post-reset speed accepted");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(sample.filtered_rpm, 1.0F),"reset clears filter history");

        const ErrorCode reconfigured = estimator.configure(EncoderSpeedConfig{60U, 16U, 2U});
        const bool baseline_after_reconfigure = estimator.has_baseline();
        const ErrorCode new_baseline = estimator.update(500U, 1.0F, sample);
        const ErrorCode new_speed = estimator.update(502U, 1.0F, sample);

        ROBOT_FOC_TEST_EXPECT(reconfigured == ErrorCode::Ok,"runtime quadrature reconfiguration succeeds");
        ROBOT_FOC_TEST_EXPECT(estimator.counts_per_revolution() == 120U,"reconfiguration updates CPR");
        ROBOT_FOC_TEST_EXPECT(!baseline_after_reconfigure,"reconfiguration clears baseline");
        ROBOT_FOC_TEST_EXPECT(new_baseline == ErrorCode::Ok,"reconfigured baseline accepted");
        ROBOT_FOC_TEST_EXPECT(new_speed == ErrorCode::Ok,"reconfigured speed accepted");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(sample.raw_rpm, 1.0F),"reconfigured RPM uses new gain");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(sample.filtered_rpm, 1.0F),"reconfiguration clears old filter history");
    }
}

void test_encoder_speed_closed()
{
    using robot_foc::algorithm::EncoderSpeedConfig;
    using robot_foc::algorithm::EncoderSpeedEstimator;
    using robot_foc::algorithm::EncoderSpeedSample;
    using robot_foc::common::ErrorCode;
    using robot_foc::test::MockEncoderCounter;
    using robot_foc::test::MockFirstOrderPlant;

    constexpr float kSamplePeriod{0.001F};
    constexpr float kPlantTimeConstant{0.02F};
    constexpr float kTargetRpm{100.0F};
    constexpr float kTargetRps{kTargetRpm / 60.0F};
    constexpr std::uint32_t kRawPpr{2500U};
    constexpr std::uint8_t kQuadratureGain{4U};
    constexpr std::uint32_t kCountsPerRevolution{kRawPpr * kQuadratureGain};
    constexpr int kSimulationSteps{350};
    constexpr int kSteadyStart{180};
    constexpr float kTrackingTolerance{2.0F};

    // 正向闭环测速。
    {
        MockFirstOrderPlant plant{kSamplePeriod, kPlantTimeConstant};
        MockEncoderCounter encoder{kCountsPerRevolution, 16U};
        EncoderSpeedEstimator estimator{};
        EncoderSpeedSample sample{};
        const ErrorCode configured = estimator.configure(EncoderSpeedConfig{kRawPpr, 16U, kQuadratureGain});

        float position_revolutions{0.0F};
        bool all_updates_ok = configured == ErrorCode::Ok;
        bool tracking_ok = true;

        for (int step = 0; step < kSimulationSteps; ++step)
        {
            const float real_rps = plant.update(kTargetRps);
            const float real_rpm = real_rps * 60.0F;
            position_revolutions += real_rps * kSamplePeriod;
            const std::uint32_t counter = encoder.update(position_revolutions);
            const ErrorCode result = estimator.update(counter, kSamplePeriod, sample);
            all_updates_ok = all_updates_ok && (result == ErrorCode::Ok);

            if (step > kSteadyStart)
            {
                tracking_ok = tracking_ok &&
                    (std::fabs(sample.filtered_rpm - real_rpm) <= kTrackingTolerance);
            }
        }

        ROBOT_FOC_TEST_EXPECT(all_updates_ok,"forward encoder closed-loop updates succeed");
        ROBOT_FOC_TEST_EXPECT(tracking_ok,"forward filtered speed tracks plant");
    }

    // 反向闭环测速。
    {
        MockFirstOrderPlant plant{kSamplePeriod, kPlantTimeConstant};
        MockEncoderCounter encoder{kCountsPerRevolution, 16U};
        EncoderSpeedEstimator estimator{};
        EncoderSpeedSample sample{};
        const ErrorCode configured = estimator.configure(
            EncoderSpeedConfig{kRawPpr, 16U, kQuadratureGain});

        float position_revolutions{0.0F};
        bool all_updates_ok = configured == ErrorCode::Ok;
        bool tracking_ok = true;

        for (int step = 0; step < kSimulationSteps; ++step)
        {
            const float real_rps = plant.update(-kTargetRps);
            const float real_rpm = real_rps * 60.0F;
            position_revolutions += real_rps * kSamplePeriod;
            const std::uint32_t counter = encoder.update(position_revolutions);
            const ErrorCode result = estimator.update(counter, kSamplePeriod, sample);
            all_updates_ok = all_updates_ok && (result == ErrorCode::Ok);

            if (step > kSteadyStart)
            {
                tracking_ok = tracking_ok &&
                    (std::fabs(sample.filtered_rpm - real_rpm) <= kTrackingTolerance);
            }
        }

        ROBOT_FOC_TEST_EXPECT(all_updates_ok,"reverse encoder closed-loop updates succeed");
        ROBOT_FOC_TEST_EXPECT(tracking_ok,"reverse filtered speed tracks plant");
    }
}
