#ifndef TEST_HELPER_HPP
#define TEST_HELPER_HPP

#include <iostream>
#include <cstdlib>

inline int test_fail_cnt = 0;

/**
 * @brief 断言底层实现，由宏自动带入文件名、行号
 * @param cond 判断条件
 * @param case_name 用例名称
 * @param file 文件路径
 * @param line 代码行号
 */
inline void expect_impl(bool cond,const char* case_name,
                        const char* file,int line){
    if (!cond){
        test_fail_cnt++;
        //std::cerr是标准错误输出流，专门输出错误信息，调用后立刻打印
        std::cerr<<"[FAIL]"<< case_name<<"|"<<file<<":"<<line<<"\n";
    }else{
        std::cout<<"[PASS]"<<case_name<<"\n";
    }
}

/**
 * @brief 断言宏
 * @param cond 判断条件
 * @param case_name 用例名称
 * @param 自动填充__FILE__,__LINE__)
 */
#define EXPECT(cond,case_name)  expect_impl((cond),(case_name),__FILE__,__LINE__)


#endif
