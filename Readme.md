# Robot_FOC
基于C++与CMake搭建的FOC电机控制器项目，面向嵌入式平台，配套PC单元测试与Python自动化测试。
> 项目定位：搭建分层可移植的FOC驱动框架，实现硬件抽象与模拟外设单元测试，嵌入式简历实战项目。

## ✨ 项目特性
1. **分层HAL硬件抽象层**
    - 顶层外设基类 `Peripheral`，统一生命周期接口 `init()` / `deinit()`
    - 禁止拷贝，RAII生命周期管理，状态保护，`[[nodiscard]]` 安全返回值
    - 所有外设驱动继承该基类，方便切换真实硬件与模拟外设
2. **模拟外设（Mock）用于PC测试**
    - MockGpio，在PC端模拟硬件行为，无需单片机即可验证业务逻辑
    - init幂等保护，析构自动执行deinit，规避资源泄漏问题
3. **跨平台编译支持**
    - CMake管理工程，区分**PC主机编译** 和 **嵌入式交叉编译**
    - PC编译启用pybind11，导出C++接口给Python做pytest自动化回归测试
    - 交叉编译固件时自动跳过pybind模块，不引入PC侧依赖
4. **单元测试方案**
    - C++原生PC测试程序 `pc_test_main.cpp`
    - pybind11绑定C++类，支持Python pytest自动化测试

## 📁 目录结构
Robot_FOC
├── Inc/ # 头文件目录
│ ├── HAL/ # HAL 硬件抽象层头文件
│ └── DRIVERS/ # 外设驱动头文件 (MockGpio 等)
├── Src/ # 源码目录
│ ├── HAL/ # Peripheral 基类实现
│ └── DRIVERS/ # MockGpio 模拟外设实现
├── test/
│ ├── pc_test_main.cpp # C++ PC 端单元测试入口
│ ├── pybind/ # pybind11 C++ 绑定代码
│ └── python/ # pytest 自动化测试脚本
├── build/ # CMake 编译输出目录（.gitignore 忽略）
├── CMakeLists.txt # 工程构建脚本
├── .gitignore # Git 忽略配置
└── README.md

