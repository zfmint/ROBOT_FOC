#include "../Inc/HAL/peripheral.hpp"
#include <iostream>

namespace robot_foc::hal{
    class DummyPeripheral : public Peripheral
    {
    private:
        /* data */
    public:
        DummyPeripheral(/* args */);
        ~DummyPeripheral();
        bool init() override{
            if (m_inited){
                std::cout<<"[DummyPeripheral] already inited\n";
                return false;
            }
            m_inited = true;
            std::cout <<"[DummyPeripheral] init success\n";
            return true;
        }

        bool deinit() override{
            if (!m_inited){
                std::cout<<"[DummyPeripheral] already deinited\n";
                return false;
            }
            m_inited = false;
            std::cout <<"[DummyPeripheral] deinit success\n";
            return true;
        }


    }; 
    DummyPeripheral::DummyPeripheral(/* args */){
    }
    DummyPeripheral::~DummyPeripheral(){
    }
}


int main(){
    using namespace robot_foc::hal;
    DummyPeripheral dev;

    std::cout<<"Before init, is_inited ="<<(dev.is_inited() ? "true" : "false") << "\n";
    
    dev.init();
    std::cout<<"After init, is_inited ="<<(dev.is_inited() ? "true" : "false") << "\n";

    dev.deinit();
    std::cout<<"After deinit, is_inited = "<<(dev.is_inited() ? "true" : "false") << "\n";

    return 0;
}
