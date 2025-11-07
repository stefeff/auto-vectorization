#include <string>
#include <immintrin.h>

auto findfirst(const std::string& s, const std::string& p)
{
    auto first1 = s.begin();
    auto last1 = s.end();
    auto first2 = p.begin();
    auto last2 = p.end();

    // logic from std_algo.h's implementation of find_first_of
    for (; first1 != last1; ++first1)
    	for (auto iter = first2; iter != last2; ++iter)
	        if (*first1 == *iter)
	            return first1;

    return last1;
}

auto findfirst_vec(const std::string& s, const std::string& p)
{
    size_t i = 0;
    for (; i + 64 <= s.size(); i += 64) {
        auto chunk = _mm512_loadu_epi8(s.data() + i);
        __mmask64 match = 0;
        for (auto c : p) {
            match = _kor_mask64(match, _mm512_cmpeq_epi8_mask(_mm512_set1_epi8(c), chunk));
        }

        if (match) [[unlikely]] {
            return s.begin() + i + _tzcnt_u64(match);
        }
    }

    auto remaining = s.size() - i;
    if (remaining) {
        auto load_mask = _cvtu64_mask64((1ull << remaining) - 1);
        auto chunk = _mm512_maskz_loadu_epi8(load_mask, s.data() + i);
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
