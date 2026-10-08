/**
 * @brief GPIO通用行为测试，仅依赖Gpio基类引用，支持任意Gpio派生实现（Mock/Real）
 * @details 验证标准HAL生命周期、configure校验、模式切换、电平读写、多态行为；
 *          使用EXPECT，单条失败继续跑完剩余用例，最后统一汇总失败计数
 * @param gpio 基类引用，外部传入具体实例
 */
#include "HAL/gpio.hpp"
#include "test_helper.hpp"
#include "COMMON/error_code.hpp"

bool run_gpio_common_tests(robot_foc::hal::Gpio& gpio)
{
    using robot_foc::hal::Gpio;
    using robot_foc::hal::GpioConfig;
    using robot_foc::hal::GpioLevel;
    using robot_foc::hal::GpioMode;
    using robot_foc::hal::GpioPull;
    using robot_foc::common::ErrorCode;

    const std::size_t fail_start = test_failure_count;
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

    // 切换为输入模式（通用HAL契约，真实硬件也需要遵守）
    auto ret_set_mode_in = gpio.set_mode(GpioMode::Input);
    EXPECT(ret_set_mode_in == ErrorCode::Ok, "GPIO switch to input mode");
    // 输入模式禁止输出电平，set_high返回ErrorUnsupported
    auto ret_set_high_input = gpio.set_high();
    EXPECT(ret_set_high_input == ErrorCode::ErrorUnsupported, "GPIO input mode rejects set_high");

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

    // ========== 基类引用多态测试（验证HAL抽象接口） ==========
    Gpio& base_gpio_ref = gpio;
    auto ret_poly_mode = base_gpio_ref.set_mode(GpioMode::OutputPushPull);
    EXPECT(ret_poly_mode == ErrorCode::Ok, "GPIO polymorphic set_mode succeeds");

    auto ret_poly_high = base_gpio_ref.set_high();
    EXPECT(ret_poly_high == ErrorCode::Ok, "GPIO polymorphic set_high succeeds");

    auto ret_poly_read = base_gpio_ref.read_level(level);
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

    return test_failure_count == fail_start;
}

/**
 * @brief MockGpio 单元测试入口
 * @details 实例创建、Mock专属注入逻辑；调用通用GPIO行为测试
 *          Mock专属：模拟外部引脚输入电平注入，仅校验Mock仿真实现
 */
#include "DRIVERS/mock_gpio.hpp"
bool test_mock_gpio()
{
    using robot_foc::drivers::MockGpio;
    using robot_foc::hal::GpioPin;
    using robot_foc::hal::GpioLevel;
    using robot_foc::hal::GpioMode;
    using robot_foc::hal::GpioConfig;
    using robot_foc::hal::GpioPull;
    using robot_foc::common::ErrorCode;

    const std::size_t fail_start = test_failure_count;

    // 【Setup：实例构造】
    MockGpio gpio{GpioPin{5U}};

    // 调用通用HAL行为测试（无mock注入逻辑）
    run_gpio_common_tests(gpio);

    // ========== Mock专属测试区域（仅Mock仿真器校验，不属于通用Gpio契约） ==========
    auto ret_deinit_common = gpio.deinit();
    EXPECT(ret_deinit_common == ErrorCode::Ok, "MockGpio common test cleanup ok");
    // 配置为输入模式
    GpioConfig input_cfg{
        GpioMode::Input,
        GpioPull::PullUp,
        GpioLevel::Low
    };
    auto ret_cfg_input = gpio.configure(input_cfg);
    EXPECT(ret_cfg_input == ErrorCode::Ok, "MockGpio configure input mode ok");

    auto ret_init_input = gpio.init();
    EXPECT(ret_init_input == ErrorCode::Ok, "MockGpio init input mode ok");

    auto ret_set_in = gpio.set_mode(GpioMode::Input);
    EXPECT(ret_set_in == ErrorCode::Ok, "MockGpio switch to input mode");

    // Mock注入动作：模拟外部硬件驱动引脚电平
    gpio.mock_force_input_level(GpioLevel::High);

    // 读取并校验注入电平
    GpioLevel lv;
    auto ret_read = gpio.read_level(lv);
    EXPECT(ret_read == ErrorCode::Ok, "MockGpio read injected level ok");
    EXPECT(lv == GpioLevel::High, "MockGpio injected input level match");

    // 输入模式禁止set_high输出
    auto ret_set_high_in = gpio.set_high();
    EXPECT(ret_set_high_in == ErrorCode::ErrorUnsupported, "MockGpio input mode reject set_high");

    auto ret_deinit_input = gpio.deinit();
    EXPECT(ret_deinit_input == ErrorCode::Ok, "MockGpio input test cleanup ok");

    return test_failure_count == fail_start;
}
