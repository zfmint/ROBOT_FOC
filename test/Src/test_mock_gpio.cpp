/**
 * @brief MockGpio 单元测试函数
 * @details 验证MockGpio完整生命周期、configure校验、模式切换、电平读写、多态行为；
 *          使用EXPECT，单条失败继续跑完剩余用例，最后统一汇总失败计数
 */
#include "DRIVERS/mock_gpio.hpp"
#include "test_helper.hpp"
#include "COMMON/error_code.hpp"

bool test_mock_gpio()
{
    using robot_foc::drivers::MockGpio;
    using robot_foc::hal::Gpio;
    using robot_foc::hal::GpioConfig;
    using robot_foc::hal::GpioLevel;
    using robot_foc::hal::GpioMode;
    using robot_foc::hal::GpioPull;
    using robot_foc::hal::GpioPin;
    using robot_foc::common::ErrorCode;

    const std::size_t fail_start = test_failure_count;

    // 构造MockGPIO实例，绑定引脚5；构造仅创建对象，不执行初始化
    MockGpio gpio{GpioPin{5U}};
    GpioLevel level{GpioLevel::Low};

    // ========== 未初始化状态测试 ==========
    // 未调用init，所有硬件操作接口都应返回非Ok错误码（ErrorNotReady）
    auto ret_set_high_pre = gpio.set_high();
    EXPECT(ret_set_high_pre == ErrorCode::ErrorNotReady, "GPIO pre-init set_high fails");

    auto ret_read_pre = gpio.read_level(level);
    EXPECT(ret_read_pre == ErrorCode::ErrorNotReady, "GPIO pre-init read_level fails");

    auto ret_mode_pre = gpio.set_mode(GpioMode::OutputPushPull);
    EXPECT(ret_mode_pre == ErrorCode::ErrorNotReady, "GPIO pre-init set_mode fails");

    // ========== configure配置校验 ==========
    // 保存原始默认配置
    // 合法配置：推挽输出，初始高电平
    GpioConfig valid_cfg{
        GpioMode::OutputPushPull,
        GpioPull::NoPull,
        GpioLevel::High
    };
    auto ret_cfg_ok = gpio.configure(valid_cfg);
    EXPECT(ret_cfg_ok == ErrorCode::Ok, "GPIO valid configure success");
    EXPECT(gpio.get_config().mode == GpioMode::OutputPushPull, "GPIO config updated after configure");

    // ========== 第一次初始化，基础功能测试 ==========
    auto ret_init1 = gpio.init();
    EXPECT(ret_init1 == ErrorCode::Ok, "GPIO first init succeeds");
    // init加载配置，初始电平为High
    auto ret_read_init = gpio.read_level(level);
    EXPECT(ret_read_init == ErrorCode::Ok,"GPIO read after init ok");
    EXPECT(level == GpioLevel::High, "GPIO init uses config init_level");

    // 已初始化状态下调用configure，拒绝修改，原配置保留
    GpioConfig new_cfg{GpioMode::Input, GpioPull::PullUp, GpioLevel::Low};
    GpioConfig after_init_cfg = gpio.get_config();
    auto ret_cfg_after_init = gpio.configure(new_cfg);
    EXPECT(ret_cfg_after_init == ErrorCode::ErrorAlreadyInit, "GPIO configure reject when initialized");
    EXPECT(gpio.get_config().mode == after_init_cfg.mode, "GPIO config unchanged after init");

    // 切换为输入模式
    auto ret_set_mode_in = gpio.set_mode(GpioMode::Input);
    EXPECT(ret_set_mode_in == ErrorCode::Ok, "GPIO switch to input mode");
    // 输入模式禁止输出电平，set_high返回ErrorUnsupported
    auto ret_set_high_input = gpio.set_high();
    EXPECT(ret_set_high_input == ErrorCode::ErrorUnsupported, "GPIO input mode rejects set_high");

    // Mock专用：外部注入输入引脚电平
    gpio.mock_force_input_level(GpioLevel::High);
    auto ret_read_inject = gpio.read_level(level);
    EXPECT(ret_read_inject == ErrorCode::Ok,"GPIO read_level after inject returns Ok");
    EXPECT(level == GpioLevel::High, "GPIO input pin level injected from outside");

    // 切回推挽输出模式
    auto ret_set_mode_out = gpio.set_mode(GpioMode::OutputPushPull);
    EXPECT(ret_set_mode_out == ErrorCode::Ok, "GPIO output push-pull mode succeeds");

    // 输出模式下设置高电平
    auto ret_set_h = gpio.set_high();
    EXPECT(ret_set_h == ErrorCode::Ok, "GPIO set_high succeeds");
    // 读取电平并校验
    auto ret_read_h = gpio.read_level(level);
    EXPECT(ret_read_h == ErrorCode::Ok, "GPIO read_level after set_high succeeds");
    EXPECT(level == GpioLevel::High, "GPIO level is high");

    // 设置低电平并校验
    auto ret_set_l = gpio.set_low();
    EXPECT(ret_set_l == ErrorCode::Ok, "GPIO set_low succeeds");
    auto ret_read_l = gpio.read_level(level);
    EXPECT(ret_read_l == ErrorCode::Ok, "GPIO read_level after set_low succeeds");
    EXPECT(level == GpioLevel::Low, "GPIO level is low");

    // ========== 基类指针多态测试（验证HAL抽象接口） ==========
    Gpio* base_gpio{&gpio};
    auto ret_poly_mode = base_gpio->set_mode(GpioMode::OutputPushPull);
    EXPECT(ret_poly_mode == ErrorCode::Ok, "GPIO polymorphic set_mode succeeds");

    auto ret_poly_high = base_gpio->set_high();
    EXPECT(ret_poly_high == ErrorCode::Ok, "GPIO polymorphic set_high succeeds");

    auto ret_poly_read = base_gpio->read_level(level);
    EXPECT(ret_poly_read == ErrorCode::Ok, "GPIO polymorphic read_level succeeds");
    EXPECT(level == GpioLevel::High, "GPIO polymorphic level is high");

    // ========== 重复初始化、反初始化生命周期测试 ==========
    // 已初始化，再次调用init返回 ErrorAlreadyInit
    auto ret_init2 = gpio.init();
    EXPECT(ret_init2 == ErrorCode::ErrorAlreadyInit, "GPIO repeated init fails");

    // 执行deinit，释放模拟外设状态
    auto ret_deinit1 = gpio.deinit();
    EXPECT(ret_deinit1 == ErrorCode::Ok, "GPIO deinit succeeds");

    // deinit之后，操作接口失效
    auto ret_post_deinit_high = gpio.set_high();
    EXPECT(ret_post_deinit_high == ErrorCode::ErrorNotReady, "GPIO post-deinit set_high fails");

    // 已经deinit，再次deinit返回 ErrorNotReady
    auto ret_deinit2 = gpio.deinit();
    EXPECT(ret_deinit2 == ErrorCode::ErrorNotReady, "GPIO repeated deinit fails");

    // ========== deinit后：不重新configure，直接init，复用原有配置 ==========
    auto ret_reinit_no_cfg = gpio.init();
    EXPECT(ret_reinit_no_cfg == ErrorCode::Ok, "GPIO re-init without re-configure after deinit");
    EXPECT(gpio.get_config().mode == GpioMode::OutputPushPull, "GPIO config preserved after deinit");

    const std::size_t fail_end = test_failure_count;
    return (fail_end == fail_start);
}