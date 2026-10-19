/**
 * @brief 滑动平均滤波单元测试函数
 * @details 对滑动平均滤波进行算法逻辑、状态机、边界行为进行验证
 */

#include <cmath>
#include <limits>

#include "test_helper.hpp"
#include "ALGORITHM/sliding_average.hpp"

void test_sliding_average()
{
    using robot_foc::algorithm::SlidingAverage;

    // ========== Case1：默认构造后不调用 reset()，初始状态校验 ==========
    {
        SlidingAverage<4U> filter{};   // 不调用 reset()
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 0U, "default ctor: valid count == 0");

        float out = 0.0F;
        bool ok = filter.calculate(5.0F, out);
        ROBOT_FOC_TEST_EXPECT(ok, "default ctor: first calc ok");
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 5.0F) < 1e-6F, "default ctor: first calc value == 5");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 1U, "default ctor: valid count == 1");
    }

    // ========== Case2：capacity() 返回编译期模板参数 ==========
    {
        SlidingAverage<4U> f4{};
        SlidingAverage<8U> f8{};
        SlidingAverage<2U> f2{};
        ROBOT_FOC_TEST_EXPECT(f4.capacity() == 4U, "capacity() == 4");
        ROBOT_FOC_TEST_EXPECT(f8.capacity() == 8U, "capacity() == 8");
        ROBOT_FOC_TEST_EXPECT(f2.capacity() == 2U, "capacity() == 2 (min)");
    }

    // ========== Case3：启动填充阶段（Capacity=4） ==========
    {
        SlidingAverage<4U> filter{};
        float out = 0.0F;

        (void)filter.calculate(1.0F, out);
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 1.0F) < 1e-6F, "stage1 avg == 1.0");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 1U, "stage1 count == 1");

        (void)filter.calculate(2.0F, out);
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 1.5F) < 1e-6F, "stage2 avg == 1.5");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 2U, "stage2 count == 2");

        (void)filter.calculate(3.0F, out);
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 2.0F) < 1e-6F, "stage3 avg == 2.0");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 3U, "stage3 count == 3");

        (void)filter.calculate(4.0F, out);
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 2.5F) < 1e-6F, "stage4 avg == 2.5 (window full)");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 4U, "stage4 count == 4");
    }

    // ========== Case4：Capacity=2 填满后继续输入，验证 FIFO ==========
    {
        SlidingAverage<2U> filter{};
        float out = 0.0F;

        (void)filter.calculate(1.0F, out);   // [1]
        (void)filter.calculate(3.0F, out);   // [1,3] full, avg=2.0
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 2.0F) < 1e-6F, "fifo2 avg == 2.0 after full");

        (void)filter.calculate(5.0F, out);  // drop 1 → [3,5], avg=4.0
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 4.0F) < 1e-6F, "fifo2 avg == 4.0 (drop 1)");

        (void)filter.calculate(7.0F, out);  // drop 3 → [5,7], avg=6.0
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 6.0F) < 1e-6F, "fifo2 avg == 6.0 (drop 3)");
    }

    // ========== Case5：多次滚动后有效数量始终等于容量 ==========
    {
        SlidingAverage<4U> filter{};
        float out = 0.0F;

        // 先填满
        for (std::size_t i = 0U; i < 4U; ++i)
        {
            (void)filter.calculate(1.0F, out);
        }
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 4U, "roll: count == capacity after fill");

        // 再滚动 10 次，count 必须始终保持 4
        for (std::size_t k = 0U; k < 10U; ++k)
        {
            (void)filter.calculate(static_cast<float>(k + 2), out);
            ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 4U, "roll: count stays == 4");
        }
    }

    // ========== Case6：reset 复位 ==========
    {
        SlidingAverage<4U> filter{};
        float out = 0.0F;
        (void)filter.calculate(1.0F, out);
        (void)filter.calculate(2.0F, out);

        filter.reset();
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 0U, "reset: count == 0");

        (void)filter.calculate(10.0F, out);
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 10.0F) < 1e-6F, "reset: avg == 10.0");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 1U, "reset: count == 1");
    }

    // ========== Case7：NaN / +Inf / -Inf 不污染窗口状态 ==========
    {
        SlidingAverage<3U> filter{};
        float out = 0.0F;

        // 先填入 [1,2,3]，sum=6，avg=2.0
        (void)filter.calculate(1.0F, out);
        (void)filter.calculate(2.0F, out);
        (void)filter.calculate(3.0F, out);
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 2.0F) < 1e-6F, "nan: baseline avg == 2.0");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 3U, "nan: baseline count == 3");

        // 连续喂入非有限值，必须全部被拒绝
        const float bad_vals[] = {
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity()
        };
        for (float v : bad_vals)
        {
            bool ok = filter.calculate(v, out);
            ROBOT_FOC_TEST_EXPECT(not ok, "nan: non-finite input rejected");
            ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 3U, "nan: count unchanged");
        }

        // 再喂一个正常值 4.0：应淘汰最老 1.0，窗口 [2,3,4]，sum=9，avg=3.0
        bool ok = filter.calculate(4.0F, out);
        ROBOT_FOC_TEST_EXPECT(ok, "nan: valid input after bad ok");
        ROBOT_FOC_TEST_EXPECT(std::isfinite(out), "nan: output still finite");
        ROBOT_FOC_TEST_EXPECT(std::fabs(out - 3.0F) < 1e-6F, "nan: avg == 3.0 (state not polluted)");
        ROBOT_FOC_TEST_EXPECT(filter.get_valid_count() == 3U, "nan: count == 3 after recovery");
    }
}
