#include "test_helper.hpp"
#include "DRIVERS/mock_gpio.hpp"
#include <iostream>

/**
 * @brief MockGpio模拟GPIO外设测试套件入口
 */
bool test_mock_gpio()
{
    std::cout << "\n===== Test Suite: MockGpio pin 5 =====" << std::endl;
    using namespace robot_foc;
    using namespace robot_foc::drivers;
    using namespace robot_foc::hal;

    MockGpio gpio(GpioPin{5});

    // ========== 场景1：未初始化时操作GPIO ==========
    bool preSetHigh = gpio.setHigh();
    std::cout << "Before init, try setHigh: " << (preSetHigh ? "OK" : "FAIL") << "\n";
    EXPECT(preSetHigh == false, "MockGpio pre-init setHigh fail");

    GpioLevel read_buf = GpioLevel::Low;
    bool preReadRet = gpio.readLevel(read_buf);
    std::cout << "Before init, try readLevel: " << (preReadRet ? "OK" : "FAIL") << "\n";
    EXPECT(preReadRet == false, "MockGpio pre-init readLevel fail");

    bool preSetMode = gpio.setMode(GpioMode::Output);
    std::cout << "Before init, try setMode: " << (preSetMode ? "OK" : "FAIL") << "\n";
    EXPECT(preSetMode == false, "MockGpio pre-init setMode fail");

    // ========== 场景2：初始化 ==========
    bool gpioInitRet = gpio.init();
    std::cout << "gpio init result: " << (gpioInitRet ? "OK" : "FAIL") << "\n";
    EXPECT(gpioInitRet == true, "MockGpio init success");

    // 场景2.1：输入模式下尝试输出（应该失败）
    bool setHighInInputMode = gpio.setHigh();
    std::cout << "Input mode try setHigh: " << (setHighInInputMode ? "OK" : "FAIL") << "\n";
    EXPECT(setHighInInputMode == false, "Input mode cannot setHigh");

    // 切换为输出模式
    bool setModeOut = gpio.setMode(GpioMode::Output);
    std::cout << "Set mode to Output: " << (setModeOut ? "OK" : "FAIL") << "\n";
    EXPECT(setModeOut == true, "SetMode Output success");

    // ========== 场景3：输出模式，读写电平 ==========
    bool retSetHigh = gpio.setHigh();
    std::cout << "setHigh: " << (retSetHigh ? "OK" : "FAIL") << "\n";
    EXPECT(retSetHigh == true, "setHigh success");

    bool retReadHigh = gpio.readLevel(read_buf);
    EXPECT(retReadHigh == true, "readLevel success after setHigh");
    EXPECT(read_buf == GpioLevel::High, "Pin level should be High");
    std::cout << "Read level after setHigh: " << (read_buf == GpioLevel::High ? "High" : "Low") << "\n";

    bool retSetLow = gpio.setLow();
    std::cout << "setLow: " << (retSetLow ? "OK" : "FAIL") << "\n";
    EXPECT(retSetLow == true, "setLow success");

    bool retReadLow = gpio.readLevel(read_buf);
    EXPECT(retReadLow == true, "readLevel success after setLow");
    EXPECT(read_buf == GpioLevel::Low, "Pin level should be Low");
    std::cout << "Read level after setLow: " << (read_buf == GpioLevel::High ? "High" : "Low") << "\n";

    // ========== 场景4：多态测试，Gpio基类指针操作 ==========
    std::cout << "\n===== Polymorphism Test (Gpio base pointer) =====" << std::endl;
    Gpio* base_gpio = &gpio;
    bool polySetMode = base_gpio->setMode(GpioMode::Output);
    EXPECT(polySetMode == true, "Polymorphism: setMode via base ptr ok");
    bool polySetHigh = base_gpio->setHigh();
    EXPECT(polySetHigh == true, "Polymorphism: setHigh via base ptr ok");

    GpioLevel poly_buf = GpioLevel::Low;
    bool polyRead = base_gpio->readLevel(poly_buf);
    EXPECT(polyRead == true, "Polymorphism: readLevel via base ptr ok");
    EXPECT(poly_buf == GpioLevel::High, "Polymorphism: level high");

    // ========== 场景5：重复init测试 ==========
    std::cout << "Before repeat init, is_initialized() = " << (gpio.is_initialized() ? "true" : "false") << "\n";

    bool repeatInitRet = gpio.init();
    std::cout << "Repeat init: " << (repeatInitRet ? "OK" : "FAIL") << "\n";
    EXPECT(repeatInitRet == false, "MockGpio repeat init return false, no state change");

    // ========== 场景6：反初始化 + 重复deinit测试 ==========
    bool deinitRet = gpio.deinit();
    std::cout << "gpio deinit result: " << (deinitRet ? "OK" : "FAIL") << "\n";
    EXPECT(deinitRet == true, "MockGpio deinit success");

    bool repeatDeinitRet = gpio.deinit();
    std::cout << "Repeat deinit: " << (repeatDeinitRet ? "OK" : "FAIL") << "\n";
    EXPECT(repeatDeinitRet == false, "MockGpio repeat deinit return false");

    return true;
}