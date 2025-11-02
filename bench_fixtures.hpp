#include "functions.hpp"

#include <benchmark/benchmark.h>

/////////////////////////////////////////////////////////////////
// Easy: Linear algebra
/////////////////////////////////////////////////////////////////

class MatrixFixture : public benchmark::Fixture {
public:

    static Matrix lhs, rhs;
    void SetUp(::benchmark::State& state);
};

/////////////////////////////////////////////////////////////////
// Medium difficulty: RGB -> Gray conversion
/////////////////////////////////////////////////////////////////

class RgbFixture : public benchmark::Fixture {
public:

    static constexpr size_t count = 0x2000;
    struct alignas(64) AlignedRGB { RGB data[count]; };
    struct alignas(64) AlignedGray { uint8_t data[count]; };

    static AlignedRGB rgb;
    static AlignedGray gray;

    void SetUp(::benchmark::State& state);
};

/////////////////////////////////////////////////////////////////
// Challenging: 2D operation
/////////////////////////////////////////////////////////////////

class FindFixture : public benchmark::Fixture {
public:

    static size_t count;
    static std::string s;
    static std::string p;
    static std::string p_long;

    void SetUp(::benchmark::State& state);
};

/////////////////////////////////////////////////////////////////
// Hard: Compression
/////////////////////////////////////////////////////////////////

class CompressFixture : public benchmark::Fixture {
public:

    static constexpr size_t count = 0x1000;
    struct alignas(64) AlignedFloats { float data[count]; };

    static AlignedFloats in;
    static AlignedFloats out;

    void SetUp(::benchmark::State& state);
};
