#include "functions.hpp"

#include <gtest/gtest.h>

constexpr size_t COUNT = 513;
constexpr size_t CAPACITY = COUNT + 64;

struct alignas(64) AlignedFloats { float data[CAPACITY]; };

static AlignedFloats input = {};
static AlignedFloats output;

template<typename Func, typename Generator>
void testCompact(Func&& func, Generator&& generator)
{
    // create inputs
    for (int i = 0; i < COUNT; ++i) {
        input.data[i] = generator();
    }

    // run tests with length COUNT .. 0
    for (int i = 0; i <= COUNT; i += 16) {
        std::fill(std::begin(output.data), std::end(output.data), 0xdeadbeef);

        auto last = func(output.data, input.data + i, COUNT - i);
        EXPECT_LE(output.data, last);
        EXPECT_LE(last, &output.data[COUNT - i]);

        auto current = &output.data[0];
        for (int k = 0; k < COUNT - i; ++k) {
            if (input.data[k + i] > 0.f) {
                EXPECT_EQ(*current, input.data[k + i]);
                ++current;
            }
        }
        EXPECT_EQ(current, last);

        for (int k = std::min<int>(current - output.data + 16, COUNT); k < CAPACITY; ++k) {
            EXPECT_FLOAT_EQ(output.data[k], 0xdeadbeef);
        }
    }
}

float allZero() {
    return 0.f;
}

float allOne() {
    return 1.f;
}

float randomValue() {
    auto r = rand();
    return r % 2 ? -r/2.f : r/2.f;
}

// Demonstrate some basic assertions.
TEST(UnitTest, CompactTest) {
    testCompact(sanitize, allZero);
    testCompact(sanitize, allOne);
    testCompact(sanitize, randomValue);
    testCompact(sanitize_vec, allZero);
    testCompact(sanitize_vec, allOne);
    testCompact(sanitize_vec, randomValue);
}

