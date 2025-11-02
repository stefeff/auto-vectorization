#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <immintrin.h>
#include <string>

/////////////////////////////////////////////////////////////////
// Easy: Linear algebra
/////////////////////////////////////////////////////////////////

// aligned storage allows for aligned load operations (usually faster)
struct alignas(64) Matrix
{
    float data[16][16];
};

// original "scalar" code
static inline Matrix mult(const Matrix& lhs, const Matrix& rhs)
{
    Matrix result;
    for (int i = 0; i < 16; ++i) {
        for (int k = 0; k < 16; ++k) {
            float sum = 0;
            for (int n = 0; n < 16; ++n) {
                sum += lhs.data[i][n] * rhs.data[n][k];
            }
            result.data[i][k] = sum;
        }
    }
    return result;
}

// optimized version, producing the same code as GCC/Clang under favorable conditions
static inline Matrix __attribute__ ((noinline)) mult_vec(const Matrix& lhs, const Matrix& rhs)
{
    Matrix result;

    const auto r0 = _mm512_load_ps(rhs.data[0]);
    const auto r1 = _mm512_load_ps(rhs.data[1]);
    const auto r2 = _mm512_load_ps(rhs.data[2]);
    const auto r3 = _mm512_load_ps(rhs.data[3]);
    const auto r4 = _mm512_load_ps(rhs.data[4]);
    const auto r5 = _mm512_load_ps(rhs.data[5]);
    const auto r6 = _mm512_load_ps(rhs.data[6]);
    const auto r7 = _mm512_load_ps(rhs.data[7]);
    const auto r8 = _mm512_load_ps(rhs.data[8]);
    const auto r9 = _mm512_load_ps(rhs.data[9]);
    const auto r10 = _mm512_load_ps(rhs.data[10]);
    const auto r11 = _mm512_load_ps(rhs.data[11]);
    const auto r12 = _mm512_load_ps(rhs.data[12]);
    const auto r13 = _mm512_load_ps(rhs.data[13]);
    const auto r14 = _mm512_load_ps(rhs.data[14]);
    const auto r15 = _mm512_load_ps(rhs.data[15]);

    for (int i = 0; i < 16; ++i) {
        // Dropping the "+ 0.0" and making the first op an ordinary multiply
        // seems to hurt performance. So, we go with a "clean" FMA chain.
        auto sum = _mm512_setzero_ps();
        sum = _mm512_fmadd_ps(r0, _mm512_set1_ps(lhs.data[i][0]), sum);
        sum = _mm512_fmadd_ps(r1, _mm512_set1_ps(lhs.data[i][1]), sum);
        sum = _mm512_fmadd_ps(r2, _mm512_set1_ps(lhs.data[i][2]), sum);
        sum = _mm512_fmadd_ps(r3, _mm512_set1_ps(lhs.data[i][3]), sum);
        sum = _mm512_fmadd_ps(r4, _mm512_set1_ps(lhs.data[i][4]), sum);
        sum = _mm512_fmadd_ps(r5, _mm512_set1_ps(lhs.data[i][5]), sum);
        sum = _mm512_fmadd_ps(r6, _mm512_set1_ps(lhs.data[i][6]), sum);
        sum = _mm512_fmadd_ps(r7, _mm512_set1_ps(lhs.data[i][7]), sum);
        sum = _mm512_fmadd_ps(r8, _mm512_set1_ps(lhs.data[i][8]), sum);
        sum = _mm512_fmadd_ps(r9, _mm512_set1_ps(lhs.data[i][9]), sum);
        sum = _mm512_fmadd_ps(r10, _mm512_set1_ps(lhs.data[i][10]), sum);
        sum = _mm512_fmadd_ps(r11, _mm512_set1_ps(lhs.data[i][11]), sum);
        sum = _mm512_fmadd_ps(r12, _mm512_set1_ps(lhs.data[i][12]), sum);
        sum = _mm512_fmadd_ps(r13, _mm512_set1_ps(lhs.data[i][13]), sum);
        sum = _mm512_fmadd_ps(r14, _mm512_set1_ps(lhs.data[i][14]), sum);
        sum = _mm512_fmadd_ps(r15, _mm512_set1_ps(lhs.data[i][15]), sum);
        _mm512_store_ps(result.data[i], sum);
    }
    return result;
}

/////////////////////////////////////////////////////////////////
// Medium difficulty: RGB -> Gray conversion
/////////////////////////////////////////////////////////////////

struct RGB
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

// original "scalar" code
// (decorators are used to separate this from the benchmarking code)
static inline void __attribute__ ((noinline)) rgb2gray(uint8_t* __restrict__ gray, const RGB* rgb, size_t n)
{
    // std::transform(rgb, rgb + n, gray, [](auto pixel) {
    //     float gr = 0.299f * pixel.red
    //              + 0.587f * pixel.green
    //              + 0.114f * pixel.blue;
    //     return static_cast<uint8_t>(gr + 0.5f);
    // });
    for (size_t i = 0; i < n; ++i) {
        auto& pixel = rgb[i];

        float gr = 0.299f * pixel.red
                 + 0.587f * pixel.green
                 + 0.114f * pixel.blue;
        gray[i] = static_cast<uint8_t>(gr + 0.5f);
    }
}

// optimized version
static inline void __attribute__ ((noinline)) rgb2gray_vec(uint8_t* gray, const RGB* rgb, size_t count)
{
    // source indexes, picking every 3rd element and place it into every 4th element
    // this is resolved at compile time and the 3 arrays are stored as constants
    uint8_t r_index_buf[64] = {};
    uint8_t g_index_buf[64] = {};
    uint8_t b_index_buf[64] = {};
    for (int i = 0; i < 16; ++i) {
        r_index_buf[i * 4] = 3 * i;
        g_index_buf[i * 4] = 3 * i + 1;
        b_index_buf[i * 4] = 3 * i + 2;
    }

    const auto r_index = _mm512_loadu_epi8(r_index_buf);
    const auto g_index = _mm512_loadu_epi8(g_index_buf);
    const auto b_index = _mm512_loadu_epi8(b_index_buf);

    // the upper 3 bytes of each 4 byte word shall be zeroed
    auto mask = _cvtu64_mask64(0x1111111111111111);

    // by default, we load and store 16 pixels (48 bytes in, 16 bytes out)
    auto load_mask = _cvtu64_mask64(0xffffffffffff);
    auto store_mask = _cvtu32_mask16(0xffff);

    // tail handling may require loading and storing 0..15 pixels
    auto to_process_last = count % 16;
    auto last_load_mask = _cvtu64_mask64((1ull << (3 * to_process_last)) - 1);
    auto last_store_mask = _cvtu32_mask16((1ull << to_process_last) - 1);

    for (size_t i = 0; i < count; i += 16) {
        // switch to shorter masks for tail handling
        if (i + 16 > count) {
            load_mask = last_load_mask;
            store_mask = last_store_mask;
        }

        // load up to 16 pixels, split into 3 channels and zero-extend values to u32
        auto chunk = _mm512_maskz_loadu_epi8(load_mask, &rgb[i]);
        auto r32 = _mm512_maskz_permutexvar_epi8(mask, r_index, chunk);
        auto g32 = _mm512_maskz_permutexvar_epi8(mask, g_index, chunk);
        auto b32 = _mm512_maskz_permutexvar_epi8(mask, b_index, chunk);

        // u32 -> fp32
        auto rf = _mm512_cvtepu32_ps(r32);
        auto gf = _mm512_cvtepu32_ps(g32);
        auto bf = _mm512_cvtepu32_ps(b32);

        // calculate gray value
        auto grayf = _mm512_fmadd_ps (rf, _mm512_set1_ps(0.299f),
                         _mm512_fmadd_ps(gf, _mm512_set1_ps(0.587f),
                             _mm512_fmadd_ps(bf, _mm512_set1_ps(0.114f),
                                _mm512_set1_ps(0.5f))));

        // fp32 -> u32 (value range is guaranteed to not exceed 0 .. 255)
        // convert u32 -> u8 by dropping the upper 24 bits
        auto gray8 = _mm512_cvtepi32_epi8(_mm512_cvttps_epu32(grayf));

        // store the resulting up to 16 bytes
        _mm_mask_storeu_epi8(&gray[i], store_mask, gray8);
    }
}

/////////////////////////////////////////////////////////////////
// Challenging: 2D operation
/////////////////////////////////////////////////////////////////

// original "scalar" code
// (decorators are used to separate this from the benchmarking code)
static inline auto __attribute__ ((noinline)) findfirst(const std::string& s, const std::string& p)
{
    // logic from std_algo.h's implementation of find_first_of
    auto first1 = s.begin();
    auto last1 = s.end();
    auto first2 = p.begin();
    auto last2 = p.end();

    for (; first1 != last1; ++first1)
    	for (auto iter = first2; iter != last2; ++iter)
	        if (*first1 == *iter)
	            return first1;

    return last1;
}

// hand-optimized version that vectorizes the inner loop
static inline auto __attribute__ ((noinline)) findfirst_inner(const std::string& s, const std::string& p)
{
    // we don't make any assumptions about the alignment of s.data() or p.data(),
    // so we use unaligned loads here

    // the last 0 .. 63 bytes of p, padded with 0
    auto tail_size = p.size() % 64;
    auto tail_mask = _cvtu64_mask64((1ull << tail_size) - 1);
    auto tail_chunk = _mm512_maskz_loadu_epi8(tail_mask, p.data() + p.size() - tail_size);

    if (p.size() < 64) {
        // for reasonably short lists of chars to find,
        // the inner loop becomes just 3 instructions: compare, check result mask, conditional jump
        return std::find_if(s.begin(), s.end(), [&](auto c) {
            return _mm512_cmpeq_epi8_mask(_mm512_set1_epi8(c), tail_chunk) != 0;
        });
    }
    else {
        // generic version: compare against 64 chars of p at a time
        // (the compiler may chose to unroll)
        return std::find_if(s.begin(), s.end(), [&](auto c) {
            auto to_find = _mm512_set1_epi8(c);

            for (size_t i = 0; i + 64 <= p.size(); i += 64) {
                if (_mm512_cmpeq_epi8_mask(to_find, _mm512_loadu_epi8(&p[i])) != 0) {
                    return true;
                }
            }

            // tail is 0 .. 63 bytes.
            // We might optimize that further by allowing tail chunks to be 64 bytes,
            // such that we never have 0 bytes here.
            return _mm512_cmpeq_epi8_mask(to_find, tail_chunk) != 0;
        });
    }
}

// hand-optimized version that vectorizes the outer loop
static inline auto __attribute__ ((noinline)) findfirst_outer(const std::string& s, const std::string& p)
{
    // Separate out the "full chunks" code path from the tail.
    // Short input strings can skip this with a single compare&jump instruction - minimal overhead.

    // Non-masked loads in this main loop are much faster on Zen4 than masked ones.
    size_t i = 0;
    for (; i + 64 <= s.size(); i += 64) {
        auto chunk = _mm512_loadu_epi8(s.data() + i);

        // aggregate mask (OR-combined) marking all chars in CHUNK that match *any* char in p
        __mmask64 match = 0;
        for (auto c : p) {
            match = _kor_mask64(match, _mm512_cmpeq_epi8_mask(_mm512_set1_epi8(c), chunk));
        }

        // report the lowest marked position
        if (match) [[unlikely]] {
            return s.begin() + i + _tzcnt_u64(match);
        }
    }

    auto remaining = s.size() - i;
    if (remaining) {
        // load remaining chars, fill remainder with 0 (will never match)
        auto load_mask = _cvtu64_mask64((1ull << remaining) - 1);
        auto chunk = _mm512_maskz_loadu_epi8(load_mask, s.data() + i);

        // perform check as in the main loop
        __mmask64 match = 0;
        for (auto c : p) {
            match = _kor_mask64(match, _mm512_cmpeq_epi8_mask(_mm512_set1_epi8(c), chunk));
        }

        if (match) [[unlikely]] {
            return s.begin() + i + _tzcnt_u64(match);
        }
    }

    return s.end();
}

// LibC-style version of findfirst (see findfirst_avx512 for implementation)
auto findfirst_avx512(const std::string& s, const std::string& p) -> std::string::const_iterator;

/////////////////////////////////////////////////////////////////
// Hard: Compression
/////////////////////////////////////////////////////////////////

// original "scalar" code
// (decorators are used to separate this from the benchmarking code)
static inline float* __attribute__ ((noinline)) sanitize(float* __restrict__ out, const float* __restrict__ in, size_t count)
{
    return std::remove_copy_if(in, in + count, out, [](auto v) { return v <= 0.f; });
}

static inline float* __attribute__ ((noinline)) sanitize_vec(float* __restrict__ out, const float* __restrict__ in, size_t count)
{
    // Although IN and OUT buffers don't overlap, the CPU seems to struggle
    // to fully hide all latencies.  Hence, we unroll 4x.

    // For maximum throughput, we also assume IN to be 64-byte aligned.
    // We also assume OUT to be large enough to receive a full copy of COUNT elements.
    size_t i = 0;
    size_t copied = 0;
    for (; i + 64 < count; i += 64) {
        // Load 64 values
        auto chunk0 = _mm512_load_ps(&in[i +  0]);
        auto chunk1 = _mm512_load_ps(&in[i + 16]);
        auto chunk2 = _mm512_load_ps(&in[i + 32]);
        auto chunk3 = _mm512_load_ps(&in[i + 48]);

        // Mark, which ones are > 0 (i.e. where 0 is less-than the value)
        auto mask0 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk0);
        auto mask1 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk1);
        auto mask2 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk2);
        auto mask3 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk3);

        // Keep those and compact the vector (fill tail with 0; we don't care)
        chunk0 = _mm512_maskz_compress_ps(mask0, chunk0);
        chunk1 = _mm512_maskz_compress_ps(mask1, chunk1);
        chunk2 = _mm512_maskz_compress_ps(mask2, chunk2);
        chunk3 = _mm512_maskz_compress_ps(mask3, chunk3);

        // Count how many element have been kept in
        // chunk0, chunk0+chunk1, chunk0+chunk1+chunk2 as well as total
        // such that we know the store offsets for each compacted chunk
        // (_mm512_kunpackw avoids shift + or)
        __mmask64 mask = mask0;
        auto to_copy0 = __builtin_popcountll(mask);
        mask = _mm512_kunpackw(mask0, mask1);
        auto to_copy1 = __builtin_popcountll(mask);
        mask = _mm512_kunpackd(mask2, mask);
        auto to_copy2 = __builtin_popcountll(mask);
        mask = mask | (__mmask64(mask3) << 48);
        auto to_copy3 = __builtin_popcountll(mask);

        // Store the compacted chunks, overlap with the tail of respective previous store.
        // Note that we don't check for OUT overflow as we assume it to have COUNT capacity.
        // Without that, we would have to do masked stores (using mask0 .. mask3) to clip
        // the tails.
        _mm512_storeu_ps(&out[copied], chunk0);
        _mm512_storeu_ps(&out[copied + to_copy0], chunk1);
        _mm512_storeu_ps(&out[copied + to_copy1], chunk2);
        _mm512_storeu_ps(&out[copied + to_copy2], chunk3);

        // advance the output pointer
        copied += to_copy3;
    }

    // tail handling of up to 63 elements (in chunks of up to 16)
    auto load_mask = _cvtu32_mask16(0xffff);
    auto to_process_last = count % 16;
    auto last_load_mask = _cvtu32_mask16((1ull << to_process_last) - 1);

    // To keep the generated code short, don't unroll anything here
    for (; i < count; i += 16) {
        if (i + 16 > count) {
            load_mask = last_load_mask;
        }

        auto chunk = _mm512_maskz_loadu_ps(load_mask, &in[i]);
        auto mask = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk);

        auto to_copy = __builtin_popcount(mask);
        chunk = _mm512_maskz_compress_ps(mask, chunk);
        _mm512_mask_storeu_ps(&out[copied], (1u << to_copy) - 1, chunk);
        copied += to_copy;
    }

    return out + copied;
}
