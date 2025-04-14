/*
 * APV2.h for apv2
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

#ifndef APV2_H_
#define APV2_H_

#include <cstdio>
#include <cstring>
#include <ctype.h>

#include "rABS.h"

#define FTOQ8(x)    ((uint8_t)((x)*256.f+0.5f))

#define APV2_LITERAL_INITIAL_PROP FTOQ8(0.43f)
#define APV2_OFFSET_INITIAL_PROP  FTOQ8(0.43f)
#define APV2_LENGTH_INITIAL_PROP  FTOQ8(0.43f)

#define APV2_M             (256)
#define APV2_L             (1<<15)
#define APV2_b             (1)
#define APV2_learning_rate (4)

#define QFACTOR (UINT32_C(1)<<16)
#define QTOF(x) ((x)/((float)QFACTOR))

extern uint32_t log2table[256];
uint32_t log2i(const uint32_t x);
void init_log2_table();
uint32_t Qlog2( uint8_t x );

class APV2 : private rABSBitDecoder<APV2_M, APV2_L, APV2_b, uint8_t, APV2_learning_rate> {
    typedef uint8_t prob_t;
public:
    APV2() : lastOffset(0), len(0) {
    }

    size_t decompress( void *dest, const void *src, size_t size ) {
        decoder_init( (uint8_t*)src, size );
        for( int i=0; i<256; ++i ) {
            literalModel[i] = APV2_LITERAL_INITIAL_PROP;
            lengthModel[i] = APV2_LENGTH_INITIAL_PROP;
        }
        for( int i=0; i<4096; ++i ) {
            offsetModel[i] = APV2_OFFSET_INITIAL_PROP;
        }
        lastOffset = 0;
        len = 0;
        uint8_t *ptr = (uint8_t*)dest;
        for(; decoder_done();) {
            uint8_t length = decode_bit_tree<8>( lengthModel );
            if( length == 0 ) {
                uint8_t lit = decode_bit_tree<8>( literalModel );
                if( ptr != NULL ) {
                    *ptr++ = lit;
                } else {
                    ++len;
                }
                continue;
            }

            uint16_t offset = decode_bit_tree<12>( offsetModel );
            if( offset == 0 ) {
                offset = lastOffset;
            }
            lastOffset = offset;

            // copy from destination
            for(; length>0; --length) {
                if( ptr != NULL ) {
                    *ptr = ptr[-lastOffset];
                    ++ptr;
                } else {
                    ++len;
                }
            }
        }
        return ptr-(uint8_t*)dest;
    }

    size_t get_data_len() {
        return len;
    }
private:
    prob_t literalModel[256];
    prob_t lengthModel[256];
    prob_t offsetModel[4096];
    int lastOffset;
    size_t len;
};

class APV2Encode : public rABSBitEncoder<APV2_M, APV2_L, APV2_b, uint8_t, APV2_learning_rate> {
    typedef uint8_t prob_t;

public:
    APV2Encode(uint8_t *data) {
        for( int i=0; i<256; ++i ) {
            literalModel[i] = APV2_LITERAL_INITIAL_PROP;
            lengthModel[i] = APV2_LENGTH_INITIAL_PROP;
        }
        for( int i=0; i<4096; ++i ) {
            offsetModel[i] = APV2_OFFSET_INITIAL_PROP;
        }

        encoder_init( data );
    }

    void encode_literal( unsigned symbol ) {
        encode_bit_tree<8>( 0, lengthModel );
        encode_bit_tree<8>( symbol, literalModel );
    }

    void encode_rep_match( unsigned length, unsigned offset ) {
        (void)(offset);
        encode_bit_tree<8>( length, lengthModel );
        encode_bit_tree<12>( 0, offsetModel );
    }

    void encode_match( unsigned length, unsigned offset ) {
        encode_bit_tree<8>( length, lengthModel );
        encode_bit_tree<12>( offset, offsetModel );
    }

private:
    prob_t literalModel[256];
    prob_t lengthModel[256];
    prob_t offsetModel[4096];
};

class APV2Cost {
    typedef uint8_t prob_t;
public:
    APV2Cost() : lastOffset( 0 ) {
        for( int i=0; i<256; ++i ) {
            literalModel[i] = APV2_LITERAL_INITIAL_PROP;
            lengthModel[i] = APV2_LENGTH_INITIAL_PROP;
        }
        for( int i=0; i<4096; ++i ) {
            offsetModel[i] = APV2_OFFSET_INITIAL_PROP;
        }
        encode = { encode_t::UNDEFINED, 0, 0 };
    }

    uint32_t inline cost_bit( unsigned sym, prob_t *probability ) {
        prob_t prob = *probability;
        assert( (prob>0) && (prob < APV2_M));
        *probability = adapt<APV2_M, APV2_learning_rate>( sym, prob );

        // prob is for symbol 0
        if( !sym ) {
            prob = APV2_M - prob; // for symbol 1
        }
        return Qlog2( prob );
    }

    template<unsigned NumBits>
    uint64_t cost_bit_tree(unsigned symbol, prob_t *probs) {
        uint64_t cost = 0;
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++) {
            unsigned bit = (symbol>>(NumBits-1)) & 1;
            symbol <<= 1;
            cost += cost_bit( bit, &probs[m] );
            m = (m << 1) + bit;
        }
        return cost;
    }

    uint64_t cost_bit_tree(unsigned symbol, unsigned NumBits, prob_t *probs) {
        uint64_t cost = 0;
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++) {
            unsigned bit = (symbol>>(NumBits-1)) & 1;
            symbol <<= 1;
            cost += cost_bit( bit, &probs[m] );
            m = (m << 1) + bit;
        }
        return cost;
    }

    uint64_t cost_literal( unsigned symbol ) {
        uint64_t cost = 0;

        cost += cost_bit_tree<8>( 0, lengthModel );
        cost += cost_bit_tree<8>( symbol, literalModel );

        encode.op = encode_t::LITERAL;
        encode.length = 0;
        encode.offset = symbol;
        return cost;
    }

    uint64_t cost_rep_match( unsigned length, int slot ) {
        (void)(slot);
        assert( slot == 0 );
        assert( length > 0 );
        uint64_t cost = 0;

        cost += cost_bit_tree<8>( length, lengthModel );
        cost += cost_bit_tree<12>( 0, offsetModel );

        encode.op = encode_t::REPEAT;
        encode.length = length;
        encode.offset = 0;
        return cost;
    }

    uint64_t cost_match( unsigned length, unsigned offset ) {
        uint64_t cost = 0;

        cost += cost_bit_tree<8>( length, lengthModel );
        lastOffset = offset;
        cost += cost_bit_tree<12>( offset, offsetModel );

        encode.op = encode_t::MATCH;
        encode.length = length;
        encode.offset = offset;
        return cost;
    }

    bool inline isRepSetEqual( APV2Cost &other ) {
        return (lastOffset == other.lastOffset)
                && (!memcmp(&encode, &other.encode, sizeof(encode_t)));
    }

    int isOffsetInSet( uint32_t value ) {
        if( value == lastOffset ) {
            return 0;
        }
        return -1;
    }

    inline uint32_t getRep( unsigned idx ) {
        (void)idx;
        return lastOffset;
    }

    void print_encoding() {
        switch( encode.op ) {
        case encode_t::LITERAL:
            printf("lit> %02X(%c)\n", encode.offset, isprint(encode.offset)?encode.offset:'.');
            break;
        case encode_t::MATCH:
            printf( "match> %d %d\n", encode.length, encode.offset );
            break;
        case encode_t::REPEAT:
            printf( "rep match> %d %d\n", encode.length, getRep(encode.offset) );
            break;
        default:
            break;
        }
    }

    typedef struct {
        enum {
            UNDEFINED = -1,
            LITERAL,
            MATCH,
            REPEAT,
        } op;
        uint8_t length;
        uint16_t offset;
    } encode_t;
    encode_t encode;
    unsigned lastOffset;

    static const int maxRepSlots = 1;
    static const int maxSlots = 61; // prime, cause we can!
    static const int maxMatch = 255;
    static const int minMatch = 1;
    static const int maxOffset = 4095;
    static const int minOffset = 1;
    static const int scoreLiteral  = 3;
    static const int scoreRepMatch = 1;
    static const int scoreMatch    = 2;
private:
    prob_t literalModel[256];
    prob_t lengthModel[256];
    prob_t offsetModel[4096];
};

size_t apv2_decoder_get_data_len(const void *src, size_t ssize);

template <class Parser>
void apv2_encode_parse( void *dest, size_t dsize, Parser& parse ) {
    uint8_t *ptr = (uint8_t*)dest;
    APV2Encode encoder(&ptr[dsize]);
    // second pass for encoding
    for ( const APV2Cost &edge : parse ) {
        const APV2Cost::encode_t &encoding = edge.encode;
        switch(encoding.op) {
            case APV2Cost::encode_t::LITERAL:
                encoder.encode_literal( encoding.offset );
                break;
            case APV2Cost::encode_t::MATCH:
                encoder.encode_match( encoding.length, encoding.offset );
                break;
            case APV2Cost::encode_t::REPEAT:
                encoder.encode_rep_match( encoding.length, encoding.offset );
                break;
            default:
                break;
        }
    }
    encoder.flush();
}

template<class Parser>
size_t apv2_encoder_get_data_len( Parser &parse ) {
    APV2Encode encoder(NULL);
    // first pass for length only
    for ( const APV2Cost &edge : parse ) {
        const APV2Cost::encode_t &encoding = edge.encode;
        switch(encoding.op) {
            case APV2Cost::encode_t::LITERAL:
                encoder.encode_literal( encoding.offset );
                break;
            case APV2Cost::encode_t::MATCH:
                encoder.encode_match( encoding.length, encoding.offset );
                break;
            case APV2Cost::encode_t::REPEAT:
                encoder.encode_rep_match( encoding.length, encoding.offset );
                break;
            default:
                break;
        }
    }
    encoder.flush();
    return encoder.get_data_len();
}

#endif /* APV2_H_ */
