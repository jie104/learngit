/**
 * @file gtest_helper
 *
 * @author pengjiali
 * @date 19-12-20.
 *
 * @describe 帮助你学习使用gtest
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <iostream>

using namespace std;

class MyTest : public testing::Test {
 protected:
    void a() {
        int i = 10;
        int j = 100;

        cout << "a" << endl;
//        ASSERT_EQ(i, j);
        cout << "a1" << endl;
    }
};

TEST_F(MyTest, MyTest) {
    // 当子函数中抛出断言，终止程序
    ASSERT_NO_FATAL_FAILURE(a());
    ASSERT_NO_FATAL_FAILURE(a());
}

TEST(a, a) {
//    EXPECT_TRUE(false);
}