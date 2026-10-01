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
    // 显式构造GpioPin，解决explicit禁止隐式转换问题
    drivers::MockGpio gpio(hal::GpioPin(5));

    bool out_level; // 用于readLevel的输出参数
    bool ret;

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
    // readLevel需要传引用变量
    ret = gpio.readLevel(out_level);
    std::cout << "setHigh, level = " << (out_level ? "High" : "Low") << "\n";
    EXPECT(retHigh == true, "MockGpio setHigh return true");
    EXPECT(ret == true, "readLevel operation success");
    EXPECT(out_level == true, "MockGpio level high after setHigh");

    bool retLow = gpio.setLow();
    ret = gpio.readLevel(out_level);
    std::cout << "setLow, level = " << (out_level ? "High" : "Low") << "\n";
    EXPECT(retLow == true, "MockGpio setLow return true");
    EXPECT(ret == true, "readLevel operation success");
    EXPECT(out_level == false, "MockGpio level low after setLow");

    // 场景4：反初始化
    auto gpioDeinitRet = gpio.deinit();
    std::cout << "gpio deinit result: " << (gpioDeinitRet ? "success" : "fail") << "\n";
    EXPECT(gpioDeinitRet == true, "MockGpio deinit ok");

    bool afterDeinitHigh = gpio.setHigh();
    std::cout << "After deinit, try setHigh: " << (afterDeinitHigh ? "OK" : "FAIL") << "\n";
    EXPECT(afterDeinitHigh == false, "MockGpio after deinit setHigh fail");

    // 场景5：重复init，验证失败不修改对象状态
    std::cout << "\n===== Repeat init test ====\n";
    bool tmpRet = gpio.init();
    EXPECT(tmpRet == true, "init");
    tmpRet = gpio.setHigh();
    EXPECT(tmpRet == true, "setHigh");
    bool repeatInitRet = gpio.init();
    ret = gpio.readLevel(out_level);
    EXPECT(repeatInitRet == true, "MockGpio second init returns true");
    EXPECT(ret == true, "readLevel operation success");
    EXPECT(out_level == true, "MockGpio repeat init does NOT change level");

    // ========== 新增：多态测试（Gpio基类指针指向MockGpio子类） ==========
    std::cout << "\n===== Polymorphism Test (Gpio base pointer) =====" << std::endl;
    robot_foc::hal::Gpio* gpio_base_ptr = new robot_foc::drivers::MockGpio(robot_foc::hal::GpioPin(7));
    bool ret_poly;
    bool out_level_poly;

    ret_poly = gpio_base_ptr->init();
    EXPECT(ret_poly == true, "Polymorphism test: init success");

    ret_poly = gpio_base_ptr->setHigh();
    EXPECT(ret_poly == true, "Polymorphism test: setHigh success");

    ret_poly = gpio_base_ptr->readLevel(out_level_poly);
    EXPECT(ret_poly == true, "Polymorphism test: readLevel call success");
    EXPECT(out_level_poly == true, "Polymorphism test: level is high");

    ret_poly = gpio_base_ptr->setLow();
    EXPECT(ret_poly == true, "Polymorphism test: setLow success");
    ret_poly = gpio_base_ptr->readLevel(out_level_poly);
    EXPECT(out_level_poly == false, "Polymorphism test: level is low");

    ret_poly = gpio_base_ptr->deinit();
    EXPECT(ret_poly == true, "Polymorphism test: deinit success");

    // 释放内存；基类析构为virtual，会自动调用MockGpio析构，执行RAII deinit
    delete gpio_base_ptr;

    return true;
}