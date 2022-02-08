#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <cmath>
#include <vector>

#include "sais.h"

static inline int log2i(int x) {
    assert(x > 0);

    return sizeof(int) * 8 - __builtin_clz(x) - 1;
}

#define Q8TOF(x)    ((x)/256.f)
#define FTOQ8(x)    ((int)((x)*256+0.5f))

template <unsigned NumBits, int (*get)(uint32_t*)>
unsigned decode_bit_tree(uint32_t* probs)
{
    unsigned m = 1;
    for (unsigned i = 0; i < NumBits; i++)
    {
        printf("%d\n", m);
        m = (m << 1) + get( &probs[m] );
    }
    return m - (1u << NumBits);
}

template<unsigned NumBits, void (*put)(bool, uint32_t*)>
void encode_bit_tree(unsigned symbol, uint32_t *probs)
{
    symbol |= 1u << NumBits;
    do
    {
        unsigned bit = symbol & 1;
        symbol >>= 1;
        put( bit, &probs[symbol] );
    } while( symbol >= 2 );
}

template<unsigned M, unsigned learning_rate>
uint32_t adapt( unsigned bit, uint32_t prob )
{
    if( bit == 1 )
    {
        unsigned adjustment = (M - prob) >> learning_rate;
        if( adjustment == 0 )
        {
            adjustment = 1;
        }
        prob += adjustment;
        if( prob > (M-1) )
        {
            prob = M-1;
        }
    } else {
        unsigned adjustment = prob >> learning_rate;
        if( adjustment == 0 )
        {
            adjustment = 1;
        }
        prob -= adjustment;
        if( prob < 1 )
        {
            prob = 1;
        }
    }
    return prob;
}


template<unsigned M, unsigned L, typename T>
class rABSEncoder {

public:
    rABSEncoder(uint8_t *data) : bits(0), numbits(0)
    {
        ptr = data-1;
        r = L;
    }

    void put(uint32_t val, int nb)
    {
        assert( nb <= 24 );
        assert( nb > 0 );
        printf("encode: %x %d\n", val&((1<<nb)-1), nb);
        bits <<= nb;
        bits |= val&((1<<nb)-1);
        assert( (numbits+nb) <= 32 );
        numbits += nb;
        while ( numbits >= 8 )
        {
            *ptr-- = (uint8_t) ( bits >> (numbits - 8) );
            numbits -= 8;
        }
        assert( numbits >= 0 && numbits < 8 );
    }

    uint8_t *_flush()
    {
        assert( numbits >= 0 && numbits < 8 );
        put( r, 16 );
        // byte align output bit stream
        int left = 7-numbits;
        put( 0, 1 );
        if( left > 0 )
            put( 0xFF, left );
        return ++ptr;
    }

    uint8_t *flush()
    {
        while( !lifo.empty() )
        {
            symbol_entry_t e = lifo.back();
            T prob = e.prob;
            _encode_bit( e.sym, prob );
            lifo.pop_back();
        }
        return _flush();
    }

    void encode_bit( unsigned symbol, T *probability )
    {
        T prob = *probability;
        lifo.push_back( { (uint_least8_t)symbol, prob } );
        *probability = adapt<M,4>( symbol, prob );
    }

    void _encode_bit( unsigned symbol, T probability )
    {
        uint32_t bias = 0;

        if( !symbol )
        {
            bias = probability;
            probability = M - probability;
        }

        // renorm
        uint32_t x = r;
        uint32_t x_max = ((L >> log2i(M)) << 1) * probability;
        for(;x>=x_max;)
        {
            put( x, 1 );
            x >>= 1;
        }

        r = ((x / probability) << log2i(M)) + (x % probability) + bias;
    }

    template<unsigned NumBits>
    void encode_bit_tree(unsigned symbol, T *probs)
    {
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++)
        {
            unsigned bit = (symbol>>(NumBits-1)) & 1;
            symbol <<= 1;
//            printf("enc context: %d\n", m);
            encode_bit( bit, &probs[m] );
            m = (m << 1) + bit;
        }
    }


private:
    typedef struct {
        uint_least8_t sym;
        T prob;
    } symbol_entry_t;

    std::vector<symbol_entry_t> lifo;
    uint32_t bits;
    int numbits;
    uint8_t *ptr;

    uint32_t r;
};


template<unsigned M, unsigned L, typename T>
class rABSDecode {

public:
    rABSDecode(const uint8_t *data) : bits(0), numbits(0)
    {
        ptr = data;
        // bit stream is byte aligned and starts with a 0 so skip 1's
        while( get(1) );
        r = get( 16 );
    }

    uint32_t get(int nb)
    {
        assert( nb <= 24 );
        assert( nb > 0 );

        while (numbits < nb)
        {
            bits |= (*ptr++) << numbits;
            numbits += 8;
        }

        uint32_t got = bits & ((1 << nb) - 1);

        bits >>= nb;
        numbits -= nb;

        printf("decode: %x %d\n", got, nb);

        return got;
    }

    // Input state x and probability for bit 1, output symbol and new state x'
    unsigned decode_bit( T *probability )
    {
        uint32_t prob = *probability;
        unsigned symbol = 0;
        uint32_t bias = 0;

        uint32_t x = r;
        // Decode symbol
        if( (x & (M - 1)) < prob )
        {
            // x is in the lower subrange; bit is 1, bias is 0
            symbol = 1;
        } else {
            // x is in the upper subrange; bit is 0, bias is probability,
            //  and probability should represent the upper subrange instead
            //  of the lower one
            bias = prob;
            prob = M - prob;
        }

        *probability = adapt<M,4>( symbol, *probability );

        // Update state
        x = prob * (x >> log2i(M)) + (x & (M - 1)) - bias;

        // Renormalize
        while( x < L ) {
            x = (x << 1) | get(1);
        }

        r = x;
        return symbol;
    }

    template <unsigned NumBits>
    unsigned decode_bit_tree(T* probs)
    {
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++)
        {
//            printf("context: %d\n", m);
            m = (m << 1) + decode_bit( &probs[m] );
        }
        return m - (1u << NumBits);
    }

private:
    uint32_t bits;
    int numbits;
    const uint8_t *ptr;

    uint32_t r;
};

typedef uint8_t prob_t;

#define APV2_INITIAL_PROP   (110)
class APV2 : private rABSDecode<256,32768,uint8_t>
{

    APV2(const uint8_t *data) : rABSDecode( data ), lastOffset(0)
    {
        for( int i=0; i<256; ++i )
        {
            literalModel[i] = APV2_INITIAL_PROP;
            lengthModel[i] = APV2_INITIAL_PROP;
        }
        for( int i=0; i<4096; ++i )
        {
            offsetModel[i] = APV2_INITIAL_PROP;
        }
    }

    void decompress( uint8_t *dest, size_t size )
    {
        uint8_t *ptr = dest;
        for( int i=0; i<size; ++i )
        {
            int8_t length = decode_bit_tree<8>( lengthModel );
            if( length == 0 )
            {
                *ptr++ = decode_bit_tree<8>( literalModel );
                continue;
            }
            int16_t offset = decode_bit_tree<12>( offsetModel );
            if( offset == 0 )
                offset = lastOffset;
            lastOffset = offset;

            // copy from destination
            uint8_t *src = ptr-offset;
            for(; length>0; --length)
            {
                *ptr++ = *src++;
            }
        }
    }

private:
    prob_t literalModel[256];
    prob_t lengthModel[256];
    prob_t offsetModel[4096];
    int lastOffset;

};

int16_t log2table[256] = { 0 };

void initQ8log2()
{
    for( int i = 1; i<256; ++i )
    {
        log2table[i] = log2( i/256.f )*256.f;
    //    printf("%d - %d\n", i, log2table[i]);
    }
}
int16_t Q8log2( uint8_t x )
{
    return log2table[x];
}

int cost_bit( bool sym, uint32_t prob )
{
    // prob is for symbol 0
    if( sym )
        prob = 256 - prob; // for symbol 1

    return -Q8log2( prob );
}


template<unsigned NumBits>
uint32_t cost_bit_tree(unsigned symbol, uint32_t *probs)
{
    uint32_t price = 0;
    symbol |= 1u << NumBits;
    do
    {
        unsigned bit = symbol & 1;
        symbol >>= 1;
        price += cost_bit( bit, probs[symbol] );

    } while( symbol >= 2 );
    return price;
}

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

void push_print_addr(bool, uint32_t *prob)
{
    printf("push> %p\n", prob);
}

int pull_print_addr(uint32_t *prob)
{
    printf("pull> %p\n", prob);
    return 0;
}

int main( int argc, char *args[] )
{
    uint8_t buf[1024] = {0};
    uint8_t data[] = { 0x12, 0x34, 0x56, 0xFF, 0xFF, 0x55, 0xF0};  //, 0x0F, 0x01, 0x02 };

    if( argc < 2 )
    {
        printf("%s <filename>\n", args[0]);
        return EXIT_SUCCESS;
    }

    initQ8log2();

    struct stat sb;
    int fd = open(args[1], O_RDONLY);
    if (fd < 0)
    {
        perror("can't open file");
        return EXIT_FAILURE;
    }

    if (fstat(fd, &sb) == -1)           /* To obtain file size */
    {
        perror("can't get file size");
        return EXIT_FAILURE;
    }
    size_t n = sb.st_size;
    assert( n <= INT32_MAX );

    uint8_t *addr = (uint8_t*)mmap(NULL, n, PROT_READ, MAP_PRIVATE, fd, 0);
    if( addr == MAP_FAILED )
    {
        perror("can't mmap file");
        return EXIT_FAILURE;
    }

    int* SA = new int[n];
    int* LCP = new int[n];
    int* ISA = new int[n];

    int r = sais(addr, SA, LCP, n );

    for( size_t i=0; i<n; ++i )
    {
        ISA[SA[i]] = i;
    }
#if 0
    for( size_t i=0; i<n; ++i )
    {
        printf("%d %s\n", LCP[i], &addr[SA[i]]);
    }
#endif
    for( size_t i=0; i<n; ++i )
    {
        int j = ISA[i];
        int lpsv = n;
        int lnsv = n;

        printf("=>%d %d\n", i, j);
        // forward scan for all matches
        for( int k=j+1; k<n; ++k )
        {
            int l = LCP[k];
            lpsv = std::min( l, lpsv );
//            printf("lpsv: %d\n", lpsv);
            if( lpsv < 3 )
                break;

            int m = SA[k];
            if( m > i )
                continue;

//            printf("%d -> %s\n", lpsv, &addr[m]);
            printf("forward (%4d|%4d)\n", m-i, lpsv);
        }
        
        // backward scan for all matches
        for( int k=j-1; k>0; --k )
        {
            int l = LCP[k+1];
            lnsv = std::min( l, lnsv );
//            printf("lnsv: %d\n", lnsv);
            if( lnsv < 3 )
                break;

            int m = SA[k];
            if( m > i )
                continue;

//            printf("%d -> %s\n", lpsv, &addr[m]);
            printf("backward (%4d|%4d)\n", m-i, lnsv);
        }
   }


    uint8_t probs[256];
    for( int i=0; i<256; ++i )
        probs[i] = 110;

    rABSEncoder<256, 32768, uint8_t> out(&buf[1024]);

    for( int j=0; j<sizeof(data); ++j )
    {
        out.encode_bit_tree<8>( data[j], probs );
    }
    uint8_t *start = out.flush();

    for( uint8_t *ptr=start; ptr<&buf[1024]; ++ptr )
    {
        printf("%02x, \n", *ptr);
    }

    for( int i=0; i<256; ++i )
        probs[i] = 110;

    rABSDecode<256, 32768, uint8_t> dec( start );

    for( int j=0; j<sizeof(data); j++ )
    {
        uint8_t value = dec.decode_bit_tree<8>( probs );
        printf("%02x\n", value);
    }

    munmap(addr, sb.st_size);
    close(fd);

    return EXIT_SUCCESS;
}


