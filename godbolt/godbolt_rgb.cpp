#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <immintrin.h>

struct RGB
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

void rgb2gray_stl(uint8_t* gray, const RGB* rgb, size_t n)
{
    std::transform(rgb, rgb + n, gray, [](auto pixel) {
        float gr = 0.299f * pixel.red
                 + 0.587f * pixel.green
                 + 0.114f * pixel.blue;
        return static_cast<uint8_t>(gr + 0.5f);
    });
}

void rgb2gray_vec(uint8_t* gray, const RGB* rgb, size_t n)
{
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

    auto mask = _cvtu64_mask64(0x1111111111111111);
    auto load_mask = _cvtu64_mask64(0xffffffffffff);
    auto store_mask = _cvtu32_mask16(0xffff);

    auto to_process_last = n % 16;
    auto last_load_mask = _cvtu64_mask64((1ull << (3 * to_process_last)) - 1);
    auto last_store_mask = _cvtu32_mask16((1ull << to_process_last) - 1);

    for (size_t i = 0; i < n; i += 16) {
        if (i + 16 > n) {
            load_mask = last_load_mask;
            store_mask = last_store_mask;
        }

        auto chunk = _mm512_maskz_loadu_epi8(load_mask, &rgb[i]);
        auto r32 = _mm512_maskz_permutexvar_epi8(mask, r_index, chunk);
        auto g32 = _mm512_maskz_permutexvar_epi8(mask, g_index, chunk);
        auto b32 = _mm512_maskz_permutexvar_epi8(mask, b_index, chunk);

        auto rf = _mm512_cvtepu32_ps(r32);
        auto gf = _mm512_cvtepu32_ps(g32);
        auto bf = _mm512_cvtepu32_ps(b32);

        auto grayf = _mm512_fmadd_ps (rf, _mm512_set1_ps(0.299f),
                         _mm512_fmadd_ps(gf, _mm512_set1_ps(0.587f),
                             _mm512_fmadd_ps(bf, _mm512_set1_ps(0.114f),
                                _mm512_set1_ps(0.5f))));

        auto gray8 = _mm512_cvtepi32_epi8(_mm512_cvttps_epu32(grayf));
        _mm_mask_storeu_epi8(&gray[i], store_mask, gray8);
    }
}
