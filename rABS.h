/*
 * rABS.h for apv2
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Dirk Helbig
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RABS_H_
#define RABS_H_

#include <cassert>
#include <cstdint>
#include <vector>

template<unsigned M, unsigned learning_rate>
inline uint32_t adapt( unsigned bit, uint32_t prob ) {
    assert(( bit == 1 ) || ( bit == 0 ));
    if( bit == 1 ) {
        unsigned adjustment = (M - prob) >> learning_rate;
        adjustment = std::max<unsigned>( adjustment, 1 );
        prob += adjustment;
        prob = std::min( prob, M-1 );
    } else {
        unsigned adjustment = prob >> learning_rate;
        adjustment = std::max<unsigned>( adjustment, 1 );
        prob -= adjustment;
        prob = std::max<unsigned>( prob, 1 );
    }
    return prob;
}

template <unsigned x>
struct static_log2i { enum { value = 1 + static_log2i<x/2>::value }; };
template <> struct static_log2i<1> { enum { value = 0 }; };

template<unsigned M, unsigned L, unsigned reNormBits, typename T, unsigned learning_rate>
class rABSBitEncoder {

public:
    rABSBitEncoder() : bits( 0 ), numbits( 0 ), ptr( NULL ), len(0), r( L ) {
    }

    void encoder_init( uint8_t *data ) {
        static_assert( static_log2i<L>::value + reNormBits <= 32 );
        bits = 0;
        numbits = 0;
        ptr = data;
        len = 0;
        r = L;
    }

    void encode_bit( unsigned symbol, T *probability ) {
        T prob = *probability;
        lifo.push_back( { symbol_entry_t::SYMBOL, (uint_least8_t)symbol, prob } );
        *probability = adapt<M, learning_rate>( symbol, prob );
    }

    uint8_t *flush() {
        while( !lifo.empty() ) {
            symbol_entry_t *e = &lifo.back();
            switch(e->op) {
            case symbol_entry_t::SYMBOL:
                _encode_bit( e->sym, e->prob );
                break;
            case symbol_entry_t::DIRECT:
                _encode_direct_bits( e->sym, e->prob );
                break;
            default:
                break;
            }
            lifo.pop_back();
        }
        return _flush();
    }

    template<unsigned NumBits>
    void encode_bit_tree(unsigned symbol, T *probs) {
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++) {
            unsigned bit = (symbol>>(NumBits-1)) & 1;
            symbol <<= 1;
            encode_bit( bit, &probs[m] );
            m = (m << 1) + bit;
        }
    }

    void encode_direct_bits( uint32_t value, T nbits ) {
        lifo.push_back( { symbol_entry_t::DIRECT, value, nbits } );
    }

    uint64_t get_data_len() {
        return len;
    }
private:

    void put(uint64_t val, int nb) {
        assert( nb <= 57 );
        assert( nb > 0 );
        bits = (bits << nb) | (val&((UINT64_C(1)<<nb)-1));

        assert( (numbits+nb) <= 64 );
        numbits += nb;

        while ( numbits >= 8 ) {
            if( ptr != NULL ) {
                // output data
                *--ptr = (uint8_t)(bits>>(numbits-8));
            } else {
                // just update encoded length
                ++len;
            }
            numbits -= 8;
        }
        assert( numbits >= 0 && numbits < 8 );
    }

    uint8_t *_flush() {
        assert( numbits >= 0 && numbits < 8 );
        put( r, static_log2i<L>::value + reNormBits );
        if( (reNormBits % 8) != 0 ) {
            // byte align output bit stream
            put( 0, 1 );
            int left = 8-numbits;
            if( left > 0 ) {
                put( 0xFF, left );
            }
        }
        assert( numbits == 0 );
        return ptr;
    }

    uint32_t renormalice( uint32_t x, uint32_t freq, uint32_t scale_bits ) {
        uint32_t x_max = ((L >> scale_bits) << reNormBits) * freq;
        for(;x>=x_max;) {
            put( x, reNormBits );
            x >>= reNormBits;
        }
        return x;
    }

    void _encode_direct_bits( uint32_t value, unsigned nbits ) {
        uint32_t x = r;
        assert( nbits <= 16 );
        x = renormalice( x, UINT32_C(1) << (16-nbits), 16);
        x = (x << nbits) | (value&((1<<nbits)-1));
        r = x;
    }

    void _encode_bit( unsigned symbol, T probability )
    {
        assert((probability > 0) && (probability < M));
        uint32_t bias = 0;

        if( !symbol ) {
            bias = probability;
            probability = M - probability;
        }

        // renorm
        uint32_t x = r;
        uint32_t x_max = ((L >> static_log2i<M>::value) << reNormBits) * probability;
        for(;x>=x_max;) {
            put( x, reNormBits );
            x >>= reNormBits;
        }

        r = ((x / probability) << static_log2i<M>::value) + (x % probability) + bias;
    }

    typedef struct {
        enum {
            SYMBOL,
            DIRECT,
        } op;
        uint32_t sym;
        T prob;
    } symbol_entry_t;

    std::vector<symbol_entry_t> lifo;
    uint64_t bits;
    int numbits;
    uint8_t *ptr;
    uint64_t len;
    uint32_t r;
};


template<unsigned M, unsigned L, unsigned reNormBits, typename T, unsigned learning_rate>
class rABSBitDecoder {

public:
    rABSBitDecoder() : bits( 0 ), numbits( 0 ), ptr( NULL ), end( NULL ), r( 0 ) {
    }

    void decoder_init( const uint8_t *data, size_t n ) {
        bits = 0;
        numbits = 0;
        ptr = data;
        end = data + n;
        // bit stream is byte aligned and starts with a 0 so skip 1's
        if( (reNormBits % 8) != 0) {
            while( get(1) );
        }
        r = get( static_log2i<L>::value + reNormBits );
    }

    bool decoder_done() {
        // all bytes read and state is back at the initial condition
        return !((ptr==end) && (r == L));
    }

    // Input state x and probability for bit 1, output symbol and new state x'
    unsigned decode_bit( T *probability ) {
        uint32_t prob = *probability;
        unsigned symbol = 0;
        uint32_t bias = 0;

        uint32_t x = r;
        // Decode symbol
        if( (x & (M - 1)) < prob ) {
            // x is in the lower subrange; bit is 1, bias is 0
            symbol = 1;
        } else {
            // x is in the upper subrange; bit is 0, bias is probability,
            //  and probability should represent the upper subrange instead
            //  of the lower one
            bias = prob;
            prob = M - prob;
        }

        *probability = adapt<M, learning_rate>( symbol, *probability );

        // Update state
        x = prob * (x >> static_log2i<M>::value) + (x & (M - 1)) - bias;

        x = renormalize( x );

        r = x;
        return symbol;
    }

    template <unsigned NumBits>
    unsigned decode_bit_tree(T* probs) {
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++) {
            m = (m << 1) + decode_bit( &probs[m] );
        }
        return m - (1u << NumBits);
    }

    uint32_t decode_direct_bits( unsigned nbits ) {
        uint32_t x = r;
        uint32_t val = x & ((UINT32_C(1) << nbits) - 1);
        x = renormalize( x >> nbits );
        r = x;
        return val;
    }

private:
    uint32_t renormalize( uint32_t x ) {
        while( x < L ) {
            x = (x << reNormBits) | get(reNormBits);
        }
        return x;
    }

    uint64_t get(int nb) {
        assert( nb <= 57 );
        assert( nb > 0 );

        while (numbits < nb) {
            bits |= (uint64_t)(*ptr++) << numbits;
            numbits += 8;
        }

        uint64_t got = bits & ((UINT64_C(1) << nb) - 1);

        bits >>= nb;
        numbits -= nb;

        return got;
    }

    uint64_t bits;
    int numbits;
    const uint8_t *ptr;
    const uint8_t *end;

    uint32_t r;
};

#endif /* RABS_H_ */
