/**
 * @brief MockGpio 单元测试函数
 * @details 验证MockGpio完整生命周期、模式切换、电平读写、多态行为；
 *          使用EXPECT，单条失败继续跑完剩余用例，最后统一汇总失败计数
 */
#include "DRIVERS/mock_gpio.hpp"
#include "test_helper.hpp"

bool test_mock_gpio()
{
    using robot_foc::drivers::MockGpio;
    using robot_foc::hal::Gpio;
    using robot_foc::hal::GpioLevel;
    using robot_foc::hal::GpioMode;
    using robot_foc::hal::GpioPin;

    // 构造MockGPIO实例，绑定引脚5；构造仅创建对象，不执行初始化
    MockGpio gpio{GpioPin{5U}};
    GpioLevel level{GpioLevel::Low};

    // ========== 未初始化状态测试 ==========
    // 未调用init，所有硬件操作接口都应直接返回false
    EXPECT(!gpio.set_high(), "GPIO pre-init set_high fails");
    EXPECT(!gpio.read_level(level), "GPIO pre-init read_level fails");
    EXPECT(!gpio.set_mode(GpioMode::Output), "GPIO pre-init set_mode fails");

    // ========== 第一次初始化，基础功能测试 ==========
    EXPECT(gpio.init(), "GPIO first init succeeds");
    // init后默认模式为Input，输入模式禁止输出电平，set_high失败
    EXPECT(!gpio.set_high(), "GPIO input mode rejects set_high");
    // 切换为输出模式
    EXPECT(gpio.set_mode(GpioMode::Output), "GPIO output mode succeeds");
    // 输出模式下设置高电平
    EXPECT(gpio.set_high(), "GPIO set_high succeeds");
    // 读取电平并校验
    EXPECT(gpio.read_level(level), "GPIO read_level after set_high succeeds");
    EXPECT(level == GpioLevel::High, "GPIO level is high");

    // 设置低电平并校验
    EXPECT(gpio.set_low(), "GPIO set_low succeeds");
    EXPECT(gpio.read_level(level), "GPIO read_level after set_low succeeds");
    EXPECT(level == GpioLevel::Low, "GPIO level is low");

    // ========== 基类指针多态测试（验证HAL抽象接口） ==========
    Gpio* base_gpio{&gpio};
    EXPECT(base_gpio->set_mode(GpioMode::Output), "GPIO polymorphic set_mode succeeds");
    EXPECT(base_gpio->set_high(), "GPIO polymorphic set_high succeeds");
    EXPECT(base_gpio->read_level(level), "GPIO polymorphic read_level succeeds");
    EXPECT(level == GpioLevel::High, "GPIO polymorphic level is high");

    // ========== 重复初始化、反初始化生命周期测试 ==========
    // 已初始化，再次调用init返回false
    EXPECT(!gpio.init(), "GPIO repeated init fails");
    // 执行deinit，释放模拟外设状态
    EXPECT(gpio.deinit(), "GPIO deinit succeeds");
    // deinit之后，操作接口失效
    EXPECT(!gpio.set_high(), "GPIO post-deinit set_high fails");
    // 已经deinit，再次deinit返回false
    EXPECT(!gpio.deinit(), "GPIO repeated deinit fails");

    return true;
}