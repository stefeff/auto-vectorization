#include "bench_fixtures.hpp"

constexpr float CPU_FREQUENCY = 4.0;

/////////////////////////////////////////////////////////////////
// Easy: Linear algebra
/////////////////////////////////////////////////////////////////

Matrix MatrixFixture::lhs = {};
Matrix MatrixFixture::rhs = {};

void MatrixFixture::SetUp(::benchmark::State& state)
{
    int v = 0;
    for (int i = 0; i < 16; ++i) {
        for (int k = 0; k < 16; ++k) {
            lhs.data[i][k] = ++v;
            rhs.data[i][k] = ++v;
        }
    }
};

Matrix __attribute__ ((noinline)) mult_no_inline(const Matrix& lhs, const Matrix& rhs)
{
    return mult(lhs, rhs);
}

BENCHMARK_F(MatrixFixture, mult_auto)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(mult_no_inline(lhs, rhs));
    }
    state.counters["Rate"] = benchmark::Counter(1, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(1 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(MatrixFixture, mult_static)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(mult(lhs, rhs));
    }
    state.counters["Rate"] = benchmark::Counter(1, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(1 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(MatrixFixture, mult_opt)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(mult_vec(lhs, rhs));
    }
    state.counters["Rate"] = benchmark::Counter(1, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(1 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}


/////////////////////////////////////////////////////////////////
// Medium difficulty: RGB -> Gray conversion
/////////////////////////////////////////////////////////////////

RgbFixture::AlignedRGB RgbFixture::rgb = {};
RgbFixture::AlignedGray RgbFixture::gray = {};

void RgbFixture::SetUp(::benchmark::State& state) {
    for (int i = 0; i < count; ++ i) {
        rgb.data[i].red = static_cast<uint8_t>(i * 3 + 0);
        rgb.data[i].green = static_cast<uint8_t>(i * 3 + 1);
        rgb.data[i].blue = static_cast<uint8_t>(i * 3 + 2);
    }
}

BENCHMARK_F(RgbFixture, rgb2gray_auto)(benchmark::State& state)
{
    for (auto _ : state) {
        rgb2gray(gray.data, rgb.data, count);
        benchmark::DoNotOptimize(gray);
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 16 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(RgbFixture, rgb2gray_opt)(benchmark::State& state)
{
    for (auto _ : state) {
        rgb2gray_vec(gray.data, rgb.data, count);
        benchmark::DoNotOptimize(gray);
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 16 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

/////////////////////////////////////////////////////////////////
// Hard: Compression
/////////////////////////////////////////////////////////////////

size_t FindFixture::count{};
std::string FindFixture::s{};
std::string FindFixture::p{};
std::string FindFixture::p_long{};

void FindFixture::SetUp(::benchmark::State& state) {
    s = std::string{"lorem ipsum dolo"};
    for (int i = 0; i < 11; ++ i) {
        s += s;
    }

    p = std::string{"\"'<>&"};
    p_long = p;
    for (int i = 128; i < 255; ++i) {
        p_long.push_back(char(i));
    }
    for (int i = 5; i < 32; ++i) {
        p_long.push_back(char(i));
    }
    count = s.size();
}

BENCHMARK_F(FindFixture, find_auto)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst(s, p));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_inner)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst_inner(s, p));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_opt)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst_outer(s, p));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_auto_long)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst(s, p_long));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_inner_long)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst_inner(s, p_long));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_opt_long)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst_outer(s, p_long));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_avx512)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst_avx512(s, p));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_avx512_long)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst_avx512(s, p_long));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

/////////////////////////////////////////////////////////////////
// Hard: Compression
/////////////////////////////////////////////////////////////////

CompressFixture::AlignedFloats CompressFixture::in = {};
CompressFixture::AlignedFloats CompressFixture::out = {};

void CompressFixture::SetUp(::benchmark::State& state) {
    for (int i = 0; i < count; ++ i) {
        in.data[i] = rand() % 2 == 0
                   ? i
                   : -i;
    }
}

BENCHMARK_F(CompressFixture, compress_auto)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(sanitize(out.data, in.data, count));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 16 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(CompressFixture, compress_opt)(benchmark::State& state)
{
    auto sum = 0;
    for (auto _ : state) {
        benchmark::DoNotOptimize(sanitize_vec(out.data, in.data, count));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 16 / CPU_FREQUENCY, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

/////////////////////////////////////////////////////////////////
// Run the benchmark
/////////////////////////////////////////////////////////////////

BENCHMARK_MAIN();
