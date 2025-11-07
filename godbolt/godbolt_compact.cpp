#include <algorithm>
#include <immintrin.h>

float* sanitize(float* __restrict__ out, const float* __restrict__ in, size_t count)
{
    return std::remove_copy_if(
        in, in + count, out,
        [](auto v) {
            return v <= 0.f;
        });
}

float* sanitize_vec(float* __restrict__ out, const float* __restrict__ in, size_t count)
{
    size_t i = 0;
    size_t copied = 0;
    for (; i + 64 < count; i += 64) {
        auto chunk0 = _mm512_load_ps(&in[i +  0]);
        auto chunk1 = _mm512_load_ps(&in[i + 16]);
        auto chunk2 = _mm512_load_ps(&in[i + 32]);
        auto chunk3 = _mm512_load_ps(&in[i + 48]);

        auto mask0 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk0);
        auto mask1 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk1);
        auto mask2 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk2);
        auto mask3 = _mm512_cmplt_ps_mask(_mm512_set1_ps(0.f), chunk3);

        chunk0 = _mm512_maskz_compress_ps(mask0, chunk0);
        chunk1 = _mm512_maskz_compress_ps(mask1, chunk1);
        chunk2 = _mm512_maskz_compress_ps(mask2, chunk2);
        chunk3 = _mm512_maskz_compress_ps(mask3, chunk3);

        __mmask64 mask = mask0;
        auto to_copy0 = __builtin_popcountll(mask);
        mask = _mm512_kunpackw(mask0, mask1);
        auto to_copy1 = __builtin_popcountll(mask);
        mask = _mm512_kunpackd(mask2, mask);
        auto to_copy2 = __builtin_popcountll(mask);
        mask = mask | (__mmask64(mask3) << 48);
        auto to_copy3 = __builtin_popcountll(mask);

        _mm512_storeu_ps(&out[copied], chunk0);
        _mm512_storeu_ps(&out[copied + to_copy0], chunk1);
        _mm512_storeu_ps(&out[copied + to_copy1], chunk2);
        _mm512_storeu_ps(&out[copied + to_copy2], chunk3);
        copied += to_copy3;
    }

    auto load_mask = _cvtu32_mask16(0xffff);
    auto to_process_last = count % 16;
    auto last_load_mask = _cvtu32_mask16((1ull << to_process_last) - 1);

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
