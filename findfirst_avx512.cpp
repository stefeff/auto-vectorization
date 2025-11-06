#include "functions.hpp"

#include <cstring>

// Utility function returning the lowest iterator where
// table[x % 128]==x, for x = *iter and table = concat(table_low, table_high)
auto find_avx512(const std::string& s, __m512i table_low, __m512i table_high)
{
    auto source = s.c_str();
    auto end = source + s.size();

    // position of the last 64-byte chunk (if any) and the corresponding read mask
    auto last = source + (s.size() & -64ul);
    __mmask64 to_read_last = _cvtu64_mask64((1ul << (end - last)) - 1);

    // To better cope with latencies, we unroll the loop to check 4 chunks in parallel
    for (; source + 256 <= end; source += 256) {
        auto chunk0 = _mm512_loadu_epi8(source);
        auto chunk1 = _mm512_loadu_epi8(source + 64);
        auto chunk2 = _mm512_loadu_epi8(source + 128);
        auto chunk3 = _mm512_loadu_epi8(source + 192);

        // Lookup each char in the table.
        auto hits0 = _mm512_permutex2var_epi8(table_low, chunk0, table_high);
        auto hits1 = _mm512_permutex2var_epi8(table_low, chunk1, table_high);
        auto hits2 = _mm512_permutex2var_epi8(table_low, chunk2, table_high);
        auto hits3 = _mm512_permutex2var_epi8(table_low, chunk3, table_high);

        // If it is a match, we hit one of the chars we are looking for.
        uint64_t match0 = _cvtmask64_u64(_mm512_cmpeq_epi8_mask(chunk0, hits0));
        uint64_t match1 = _cvtmask64_u64(_mm512_cmpeq_epi8_mask(chunk1, hits1));
        uint64_t match2 = _cvtmask64_u64(_mm512_cmpeq_epi8_mask(chunk2, hits2));
        uint64_t match3 = _cvtmask64_u64(_mm512_cmpeq_epi8_mask(chunk3, hits3));

        // Quickly check if there is any hit
        if (match0 | match1 | match2 | match3) [[unlikely]] {
            // Return the first one
            if (match0) {
                return std::string::const_iterator{source + _tzcnt_u64(match0)};
            }
            if (match1) {
                return std::string::const_iterator{source + 64 + _tzcnt_u64(match1)};
            }
            if (match2) {
                return std::string::const_iterator{source + 128 + _tzcnt_u64(match2)};
            }
            return std::string::const_iterator{source + 192 + _tzcnt_u64(match3)};
        }
    }

    for (; source < end; source += 64) {
        // Read chunk, pad last chunk with trailing 0 if needed
        auto chunk = source == last
                    ? _mm512_maskz_loadu_epi8(to_read_last, source)
                    : _mm512_loadu_epi8(source);

        // Compare against table.
        // If 0 is in table and this chunk has been padded with trailing 0,
        // we will get at hit at s.end() - which is the correct result in case
        // there is no earlier hit.
        auto hits = _mm512_permutex2var_epi8(table_low, chunk, table_high);
        auto match = _mm512_cmpeq_epi8_mask(chunk, hits);
        if (match) {
            return std::string::const_iterator{source + _tzcnt_u64(match)};
        }
    }

    // No match found
    return s.end();
}

// Utility function returning the lowest iterator where
// table[x]==x, for x = *iter and table = concat(table0, table1, table2, table3)
auto find_avx512(const std::string& s, __m512i table0, __m512i table1, __m512i table2, __m512i table3)
{
    auto source = s.c_str();
    auto end = source + s.size();

    // position of the last 64-byte chunk (if any) and the corresponding read mask
    auto last = source + (s.size() & -64ul);
    __mmask64 to_read_last = _cvtu64_mask64((1ul << (end - last)) - 1);

    // To better cope with latencies, we unroll the loop to check 4 chunks in parallel
    for (; source + 256 <= end; source += 256) {
        auto chunk0 = _mm512_loadu_epi8(source);
        auto chunk1 = _mm512_loadu_epi8(source + 64);
        auto chunk2 = _mm512_loadu_epi8(source + 128);
        auto chunk3 = _mm512_loadu_epi8(source + 192);

        // Lookup each char in the table (two lookups required per chunk to cover the whole table).
        auto hits0_01 = _mm512_permutex2var_epi8(table0, chunk0, table1);
        auto hits1_01 = _mm512_permutex2var_epi8(table0, chunk1, table1);
        auto hits2_01 = _mm512_permutex2var_epi8(table0, chunk2, table1);
        auto hits3_01 = _mm512_permutex2var_epi8(table0, chunk3, table1);
        auto hits0_23 = _mm512_permutex2var_epi8(table2, chunk0, table3);
        auto hits1_23 = _mm512_permutex2var_epi8(table2, chunk1, table3);
        auto hits2_23 = _mm512_permutex2var_epi8(table2, chunk2, table3);
        auto hits3_23 = _mm512_permutex2var_epi8(table2, chunk3, table3);

        // If it is a match, we hit one of the chars we are looking for.
        uint64_t match0 = _cvtmask64_u64(_kor_mask64(_mm512_cmpeq_epi8_mask(chunk0, hits0_01), _mm512_cmpeq_epi8_mask(chunk0, hits0_23)));
        uint64_t match1 = _cvtmask64_u64(_kor_mask64(_mm512_cmpeq_epi8_mask(chunk1, hits1_01), _mm512_cmpeq_epi8_mask(chunk1, hits1_23)));
        uint64_t match2 = _cvtmask64_u64(_kor_mask64(_mm512_cmpeq_epi8_mask(chunk2, hits2_01), _mm512_cmpeq_epi8_mask(chunk2, hits2_23)));
        uint64_t match3 = _cvtmask64_u64(_kor_mask64(_mm512_cmpeq_epi8_mask(chunk3, hits3_01), _mm512_cmpeq_epi8_mask(chunk3, hits3_23)));

        // Quickly check if there is any hit
        if (match0 | match1 | match2 | match3) [[unlikely]] {
            // Return the first one
            if (match0) {
                return std::string::const_iterator{source + _tzcnt_u64(match0)};
            }
            if (match1) {
                return std::string::const_iterator{source + 64 + _tzcnt_u64(match1)};
            }
            if (match2) {
                return std::string::const_iterator{source + 128 + _tzcnt_u64(match2)};
            }
            return std::string::const_iterator{source + 192 + _tzcnt_u64(match3)};
        }
    }

    for (; source < end; source += 64) {
        // Read chunk, pad last chunk with trailing 0 if needed
        auto chunk = source == last
                    ? _mm512_maskz_loadu_epi8(to_read_last, source)
                    : _mm512_loadu_epi8(source);

        // Compare against table.
        // If 0 is in table and this chunk has been padded with trailing 0,
        // we will get at hit at s.end() - which is the correct result in case
        // there is no earlier hit.
        auto hits_01 = _mm512_permutex2var_epi8(table0, chunk, table1);
        auto hits_23 = _mm512_permutex2var_epi8(table2, chunk, table3);
        auto match = _kor_mask64(_mm512_cmpeq_epi8_mask(chunk, hits_01), _mm512_cmpeq_epi8_mask(chunk, hits_23));
        if (match) {
            return std::string::const_iterator{source + _tzcnt_u64(match)};
        }
    }

    // No match found
    return s.end();
}

// LibC-style implementation of findfirst
auto findfirst_avx512(const std::string& s, const std::string& p) -> std::string::const_iterator
{
    // Building the lookup tables is expensive.
    // This metric picks a reasonable threshold.
    if (s.size() * p.size() > 2500) {
        // We calculate a lookup table to_find[c] == c, iff c in P.

        // If the interface is changed appropriately, constant propagation might allow the
        // compiler to determine the contents of this array at compile time and hard-code it.

        char to_find[256] = {};
        to_find[0] = 1; // we want to detect any 0 within p

        // Filling the table is performance critical unless S is at least 1000x as long as P
        auto count = p.size();
        size_t i = 0;
        for (; i + 4 < count; i += 4) {
            // Load into e.g. AX, so c0 and c1 are readily available in AL, AH
            std::uint16_t c01;
            std::memcpy(&c01, &p[i], 2);
            std::uint16_t c23;
            std::memcpy(&c23, &p[i + 2], 2);

            auto c0 = (unsigned char)c01;
            auto c1 = (unsigned char)(c01 >> 8);
            auto c2 = (unsigned char)c23;
            auto c3 = (unsigned char)(c23 >> 8);

            to_find[c0] = c0;
            to_find[c1] = c1;
            to_find[c2] = c2;
            to_find[c3] = c3;
        }

        // tail handling, if needed
        for (; i < count; ++i) {
            unsigned char c = p[i];
            to_find[c] = c;
        }

        // to_find[x] == x, iff x in P
        auto table0 = _mm512_loadu_epi8(&to_find[0]);
        auto table1 = _mm512_loadu_epi8(&to_find[64]);
        auto table2 = _mm512_loadu_epi8(&to_find[128]);
        auto table3 = _mm512_loadu_epi8(&to_find[192]);

        // If 0 is in P, make table[0][0] != 0 such that will be seen as "set"
        auto table1_marked0 = _mm512_sub_epi8(table0, _mm512_set_epi64(0, 0, 0, 0, 0, 0, 0, 1));

        // 256 flags indicating which table entries have been set (i.e. occurred in P)
        auto is_set0 = _mm512_cmpneq_epi8_mask(_mm512_setzero_si512(), table1_marked0);
        auto is_set1 = _mm512_cmpneq_epi8_mask(_mm512_setzero_si512(), table1);
        auto is_set2 = _mm512_cmpneq_epi8_mask(_mm512_setzero_si512(), table2);
        auto is_set3 = _mm512_cmpneq_epi8_mask(_mm512_setzero_si512(), table3);

        // Quick path: in many cases, the lower 7 bits of all chars in P are unique
        // In that case, we could merge the upper 128 table entries into the lower 128.
        bool conflict = ((is_set0 & is_set2) | (is_set1 & is_set3)) != 0;
        if (conflict) {
            // We need the full 256-entry table (2x the lookup cost)
            return find_avx512(s, table0, table1, table2, table3);
        }
        else {
            // A 128-entry table is enough
            return find_avx512(s, table0 | table2, table1 | table3);
        }
    }
    // pesky edge-case.
    else if (p.empty()) {
        return s.end();
    }
    else {
        // fall back to direct matching function for small-ish S * P combos
        return findfirst_outer(s, p);
    }
}
