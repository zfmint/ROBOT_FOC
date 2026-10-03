/**
 * @brief DummyPeripheral 外设基类单元测试
 * @details 测试顶层Peripheral基类生命周期：init/deinit状态管理、重复init/deinit保护；
 *          用最小派生类DummyPeripheral验证基类is_initialized、mark_initialized/mark_deinitialized逻辑
 */
#include "HAL/peripheral.hpp"
#include "test_helper.hpp"

namespace robot_foc::test
{

/**
 * @brief 空外设派生类，仅实现init/deinit纯虚函数，用于测试Peripheral基类逻辑
 */
class DummyPeripheral final : public robot_foc::hal::Peripheral
{
public:
    [[nodiscard]] bool init() noexcept override
    {
        return mark_initialized();
    }

    [[nodiscard]] bool deinit() noexcept override
    {
        return mark_deinitialized();
    }
};

} // namespace robot_foc::test

bool test_dummy_peripheral()
{
    using robot_foc::test::DummyPeripheral;

    DummyPeripheral device;
    // 构造完成默认未初始化
    EXPECT(!device.is_initialized(), "Dummy is not initialized before init");
    // 第一次初始化成功，状态切换为已初始化
    EXPECT(device.init(), "Dummy first init succeeds");
    EXPECT(device.is_initialized(), "Dummy is initialized after init");
    // 重复调用init，状态不变，返回false
    EXPECT(!device.init(), "Dummy repeated init fails");
    // 执行反初始化，状态切换为未初始化
    EXPECT(device.deinit(), "Dummy deinit succeeds");
    EXPECT(!device.is_initialized(), "Dummy is not initialized after deinit");
    // 重复deinit，返回false
    EXPECT(!device.deinit(), "Dummy repeated deinit fails");

    return true;
}