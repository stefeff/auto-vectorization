#include "functions.hpp"

BENCHMARK_F(MatrixFixture, mult_scalar)(benchmark::State& state)
{
    // Perform setup here
    for (auto _ : state) {
        // This code gets timed
        benchmark::DoNotOptimize(mult(lhs, rhs));
    }
    state.counters["Rate"] = benchmark::Counter(1, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(1 / 5.2, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}


BENCHMARK_F(RgbFixture, rgb2gray_scalar)(benchmark::State& state)
{
    // Perform setup here
    for (auto _ : state) {
        // This code gets timed
        rgb2gray(gray.data, rgb.data, count);
        benchmark::DoNotOptimize(gray);
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 16 / 5.2, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
    state.counters["Cycl1"] = benchmark::Counter(count / 5.2, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_scalar)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst(s, p));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / 5.2, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(FindFixture, find_scalar_long)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(findfirst(s, p_long));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 64 / 5.2, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

BENCHMARK_F(CompressFixture, compress_scalar)(benchmark::State& state)
{
    for (auto _ : state) {
        benchmark::DoNotOptimize(sanitize(out.data, in.data, count));
    }
    state.counters["Rate"] = benchmark::Counter(count, benchmark::Counter::kIsIterationInvariantRate);
    state.counters["Cycl"] = benchmark::Counter(count / 16 / 5.2, benchmark::Counter::kIsIterationInvariantRate | benchmark::Counter::kInvert);
}

