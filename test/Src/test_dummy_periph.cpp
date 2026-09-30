#include "test_helper.hpp"
#include "HAL/peripheral.hpp"

namespace robot_foc::hal{
    /**
     * @brief 测试替身DummyPeripheral，验证Peripheral基类状态管理
     */
    class DummyPeripheral : public Peripheral
    {
    public:
        bool init() noexcept override
        {
            if(mark_initialized())
            {
                std::cout <<"[DummyPeripheral] init success\n";
                return true;
            }
            std::cout<<"[DummyPeripheral] already inited\n";
            return false;
        }

        bool deinit() noexcept override
        {
            if(mark_deinitialized())
            {
                std::cout <<"[DummyPeripheral] deinit success\n";
                return true;
            }
            std::cout<<"[DummyPeripheral] already deinited\n";
            return false;
        }
    }; 
}

/**
 * @brief DummyPeripheral测试套件入口
 */
bool test_dummy_peripheral()
{
    std::cout << "\n===== Test Suite: DummyPeripheral =====" << std::endl;
    using namespace robot_foc::hal;
    DummyPeripheral dev;

    EXPECT(dev.is_initialized() == false, "Dummy before init not inited");
    
    bool retInit = dev.init();
    EXPECT(retInit == true, "Dummy first init success");
    EXPECT(dev.is_initialized() == true, "Dummy after init marked inited");

    bool retDeinit = dev.deinit();
    EXPECT(retDeinit == true, "Dummy deinit success");
    EXPECT(dev.is_initialized() == false, "Dummy after deinit not inited");

    return true;
}