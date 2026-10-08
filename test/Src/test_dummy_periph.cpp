/**
 * @brief DummyPeripheral 外设基类单元测试
 * @details 测试顶层Peripheral基类生命周期：init/deinit状态管理、重复init/deinit保护；
 *          用最小派生类DummyPeripheral验证基类is_initialized、mark_initialized/mark_deinitialized逻辑
 */
#include "HAL/peripheral.hpp"
#include "test_helper.hpp"
#include "COMMON/error_code.hpp"

namespace robot_foc::test
{

/**
 * @brief 空外设派生类，仅实现init/deinit纯虚函数，用于测试Peripheral基类逻辑
 */
class DummyPeripheral final : public robot_foc::hal::Peripheral
{
public:
    [[nodiscard]] robot_foc::common::ErrorCode init() noexcept override
    {
        return mark_initialized();
    }

    [[nodiscard]] robot_foc::common::ErrorCode deinit() noexcept override
    {
        return mark_deinitialized();
    }
};

} // namespace robot_foc::test

bool test_dummy_peripheral()
{
    using robot_foc::test::DummyPeripheral;
    using robot_foc::common::ErrorCode;
    const std::size_t fail_start = test_failure_count;

    DummyPeripheral device;
    // 构造完成默认未初始化
    EXPECT(!device.is_initialized(), "Dummy is not initialized before init");

    // 第一次初始化成功，返回Ok，状态切换为已初始化
    auto ret_init1 = device.init();
    EXPECT(ret_init1 == ErrorCode::Ok, "Dummy first init succeeds");
    EXPECT(device.is_initialized(), "Dummy is initialized after init");

    // 重复调用init，返回 ErrorAlreadyInit
    auto ret_init2 = device.init();
    EXPECT(ret_init2 == ErrorCode::ErrorAlreadyInit, "Dummy repeated init fails");
    EXPECT(device.is_initialized(), "Dummy still initialized after duplicate init");

    // 执行反初始化，返回Ok
    auto ret_deinit1 = device.deinit();
    EXPECT(ret_deinit1 == ErrorCode::Ok, "Dummy deinit succeeds");
    EXPECT(!device.is_initialized(), "Dummy is not initialized after deinit");

    // 重复deinit：设备已经未初始化，返回 ErrorNotReady
    auto ret_deinit2 = device.deinit();
    EXPECT(ret_deinit2 == ErrorCode::ErrorNotReady, "Dummy repeated deinit fails");

    // 多实例状态隔离校验
    DummyPeripheral dev2;
    EXPECT(!dev2.is_initialized(), "dev2 separate instance not init");
    auto ret_dev2_init = dev2.init();
    EXPECT(ret_dev2_init == ErrorCode::Ok, "dev2 init ok");
    EXPECT(dev2.is_initialized(), "dev2 initialized");
    EXPECT(!device.is_initialized(), "original device state unaffected");

    const std::size_t fail_end = test_failure_count;
    return (fail_end == fail_start);
}