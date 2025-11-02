#include "functions.hpp"

#include <gtest/gtest.h>

template<typename Func>
void testFind(Func&& func)
{
    const std::string empty{};
    const std::string one{"1"};

    const std::string tiny{"this is a tiny string"};
    const std::string large{"Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.?"};
    const std::string nontext{"1234567890!£$%^&*(){}[]1234567890!£$%^&*(){}[]1234567890!£$%^&*(){}[]1234567890!£$%^&*(){}[]1234567890!£$%^&*(){}[]"};

    EXPECT_EQ(func(empty, ""), empty.end());
    EXPECT_EQ(func(empty, "1"), empty.end());
    EXPECT_EQ(func(empty, large), empty.end());

    EXPECT_EQ(func(one, ""), one.end());
    EXPECT_EQ(func(one, "A"), one.end());
    EXPECT_EQ(func(one, large), one.end());
    EXPECT_EQ(func(one, "1"), one.begin());
    EXPECT_EQ(func(one, large + "1"), one.begin());

    EXPECT_EQ(func(tiny, ""), tiny.end());
    EXPECT_EQ(func(tiny, "A"), tiny.end());
    EXPECT_EQ(func(tiny, large), tiny.begin());
    EXPECT_EQ(func(tiny, "1234g"), tiny.end() - 1);

    EXPECT_EQ(func(large, ""), large.end());
    EXPECT_EQ(func(large, "A"), large.end());
    EXPECT_EQ(func(large, nontext), large.end());
    EXPECT_EQ(func(large, "L"), large.begin());
    EXPECT_EQ(func(large, ","), large.begin() + 26);
    EXPECT_EQ(func(large, "?"), large.end() - 1);
    EXPECT_EQ(func(large, nontext + "L"), large.begin());
    EXPECT_EQ(func(large, nontext + ","), large.begin() + 26);
    EXPECT_EQ(func(large, nontext + "?"), large.end() - 1);
}

// Demonstrate some basic assertions.
TEST(UnitTest, FindTest) {
    testFind(findfirst);
    testFind(findfirst_inner);
    testFind(findfirst_outer);
    testFind(findfirst_avx512);
}

