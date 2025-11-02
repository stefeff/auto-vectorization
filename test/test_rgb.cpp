#include "functions.hpp"

#include <gtest/gtest.h>

template<typename Func>
void testRGB(Func&& func)
{
    constexpr size_t COUNT = 1000;
    constexpr size_t CAPACITY = COUNT + 64;

    RGB input[CAPACITY] = {};
    std::uint8_t output[CAPACITY];

    // create inputs
    for (int i = 0; i < COUNT; ++i) {
        input[i].red = i % 256;
        input[i].green = 255 - ((i / 5) % 256);
        input[i].blue = (3 * i) % 256;
    }

    // run tests with length COUNT .. 0
    for (int i = 0; i <= COUNT; ++i) {
        std::fill(std::begin(output), std::end(output), 0xff);
        func(output, input + i, COUNT - i);

        for (int k = 0; k < COUNT - i; ++k) {
            auto pixel = input[k + i];
            float gr = 0.299f * pixel.red
                     + 0.587f * pixel.green
                     + 0.114f * pixel.blue;
            EXPECT_EQ(output[k], static_cast<uint8_t>(gr + 0.5f));
        }

        for (int k = COUNT - i; k < CAPACITY; ++k) {
            EXPECT_EQ(output[k], 0xff);
        }
    }
}

// Demonstrate some basic assertions.
TEST(UnitTest, RgbTest) {
    testRGB(rgb2gray);
    testRGB(rgb2gray_vec);
}

