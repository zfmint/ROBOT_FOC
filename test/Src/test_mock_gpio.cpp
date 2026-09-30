#include "test_helper.hpp"
#include "DRIVERS/mock_gpio.hpp"

/**
 * @brief MockGpio模拟GPIO外设测试套件入口
 */
bool test_mock_gpio()
{
    std::cout << "\n===== Test Suite: MockGpio pin 5 =====" << std::endl;
    using namespace robot_foc::drivers;
    MockGpio gpio(5);

    // 场景1：未初始化时操作GPIO
    bool preSetHigh = gpio.setHigh();
    std::cout << "Before init, try setHigh: " << (preSetHigh ? "OK" : "FAIL") << "\n";
    EXPECT(preSetHigh == false, "MockGpio pre-init setHigh fail");

    // 场景2：初始化
    auto gpioInitRet = gpio.init();
    std::cout << "gpio init result: " << (gpioInitRet ? "success" : "fail") << "\n";
    EXPECT(gpioInitRet == true, "MockGpio first init ok");

    // 场景3：高低电平读写
    bool retHigh = gpio.setHigh();
    std::cout << "setHigh, level = " << (gpio.readLevel() ? "High" : "Low") << "\n";
    EXPECT(retHigh == true, "MockGpio setHigh return true");
    EXPECT(gpio.readLevel() == true, "MockGpio level high after setHigh");

    bool retLow = gpio.setLow();
    std::cout << "setLow, level = " << (gpio.readLevel() ? "High" : "Low") << "\n";
    EXPECT(retLow == true, "MockGpio setLow return true");
    EXPECT(gpio.readLevel() == false, "MockGpio level low after setLow");

    // 场景4：反初始化
    auto gpioDeinitRet = gpio.deinit();
    std::cout << "gpio deinit result: " << (gpioDeinitRet ? "success" : "fail") << "\n";
    EXPECT(gpioDeinitRet == true, "MockGpio deinit ok");

    bool afterDeinitHigh = gpio.setHigh();
    std::cout << "After deinit, try setHigh: " << (afterDeinitHigh ? "OK" : "FAIL") << "\n";
    EXPECT(afterDeinitHigh == false, "MockGpio after deinit setHigh fail");

    // 场景5：重复init，验证失败不修改对象状态
    std::cout << "\n===== Repeat init test ====\n";
    gpio.init();
    gpio.setHigh();
    bool repeatInitRet = gpio.init();
    EXPECT(repeatInitRet == false, "MockGpio second init returns false");
    EXPECT(gpio.readLevel() == true, "MockGpio repeat init does NOT change level");

    return true;
}