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
            symbol_entry_t *e = &lifo.back();
            T prob = e->prob;
            _encode_bit( e->sym, prob );
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

//        printf("decode: %x %d\n", got, nb);

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

#define APV2_INITIAL_PROP   FTOQ8(0.43f)
class APV2 : private rABSDecode<256,32768,uint8_t>
{
public:
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
        printf("%s\n", __FUNCTION__);
        uint8_t *ptr = dest;
        for( int i=0; i<size; ++i )
        {
            int8_t length = decode_bit_tree<8>( lengthModel );
            printf("length: %3d ", length);
            if( length == 0 )
            {
                uint8_t lit = decode_bit_tree<8>( literalModel );
                *ptr++ = lit;
                printf("lit: %02d\n", lit);
                continue;
            }
            int16_t offset = decode_bit_tree<12>( offsetModel );
            printf("offset: %4d\n", offset);
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

class APV2Encode : public rABSEncoder<256,32768,uint8_t>
{

public:
    APV2Encode(uint8_t *data) : rABSEncoder( data ), lastOffset( 0 )
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

    void encode_literal( unsigned symbol )
    {
        encode_bit_tree<8>( 0, lengthModel );
        encode_bit_tree<8>( symbol, literalModel );
    }

    void encode_match( unsigned length, unsigned offset )
    {
        encode_bit_tree<8>( length, lengthModel );
        unsigned encode_offset = offset;
        if( lastOffset == offset )
            encode_offset = 0;
        lastOffset = offset;
        encode_bit_tree<12>( encode_offset, offsetModel );
    }

private:

    prob_t literalModel[256];
    prob_t lengthModel[256];
    prob_t offsetModel[4096];
    int lastOffset;

};

template<typename T>
class APV2Cost
{

public:
    APV2Cost() : lastOffset( 0 ), price( SIZE_MAX ), link( -1 ), slot( -1 )
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

    int cost_bit( unsigned sym, T *probability )
    {
        T prob = *probability;
        *probability = adapt<256, 4>( sym, prob );

        // prob is for symbol 0
        if( !sym )
            prob = 256 - prob; // for symbol 1

        return -Q8log2( prob );
    }

    template<unsigned NumBits>
    uint32_t cost_bit_tree(unsigned symbol, T *probs)
    {
        uint32_t cost = 0;
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++)
        {
            unsigned bit = (symbol>>(NumBits-1)) & 1;
            symbol <<= 1;
//            printf("enc context: %d\n", m);
            cost += cost_bit( bit, &probs[m] );
            m = (m << 1) + bit;
        }
        return cost;
    }

    uint32_t cost_literal( unsigned symbol )
    {
        uint32_t cost = 0;
        cost += cost_bit_tree<8>( 0, lengthModel );
        cost += cost_bit_tree<8>( symbol, literalModel );
        encode.lenght = 0;
        encode.offset = symbol;
        return cost;
    }

    uint32_t cost_match( unsigned length, unsigned offset )
    {
        uint32_t cost = 0;
        cost += cost_bit_tree<8>( length, lengthModel );
        unsigned encode_offset = offset;
        if( lastOffset == offset )
            encode_offset = 0;
        lastOffset = offset;
        cost += cost_bit_tree<12>( encode_offset, offsetModel );
        encode.lenght = length;
        encode.offset = encode_offset;
        return cost;
    }

    void print_encoding()
    {
        if( encode.lenght == 0 )
        {
            printf("lit> %02x(%c)\n", encode.offset, encode.offset);
            return;
        }
        printf( "match> %d %d\n", encode.lenght, encode.offset );
    }

    template<unsigned NUM_SLOTS>
    void update_forward_arrival( APV2Cost cost_slot[][NUM_SLOTS], size_t p, size_t length, size_t offset )
    {
        size_t np = p + length;
        for(uint8_t from_slot = 0; from_slot<NUM_SLOTS; ++from_slot )
        {
            // skip if no valid estimate for current slot
            if( cost_slot[p][from_slot].price == SIZE_MAX )
                break;

            APV2Cost tmp = cost_slot[p][from_slot];
            uint32_t match_cost = tmp.cost_match( length, offset );
            if( cost_slot[np][NUM_SLOTS-1].price >
                cost_slot[p][from_slot].price + match_cost )
            {
                tmp.price = cost_slot[p][from_slot].price + match_cost;
                tmp.link = p;
                tmp.slot = from_slot;
                // update statistics, keep slots sorted by price
                int slot;
                for (slot = NUM_SLOTS - 2; (slot >= 0 && cost_slot[np][slot].price > tmp.price); slot--)
                    cost_slot[np][slot + 1] = cost_slot[np][slot];
                printf("from_slot: %d slot: %d\n", from_slot, slot+1);
//                        printf("match slot: %d\n", slot);
                cost_slot[np][slot+1] = tmp;
            }
        }

    }

    void update_forward_arrival( size_t p, uint8_t literal )
    {

    }

    size_t price;
    ssize_t link;
    int8_t slot;
    struct {
        uint8_t lenght;
        uint16_t offset;
    } encode;

private:

    prob_t literalModel[256];
    prob_t lengthModel[256];
    prob_t offsetModel[4096];
    int lastOffset;

};

template<unsigned ARRAY_SIZE, typename T>
void insert_sorted( APV2Cost<T> array[], APV2Cost<T> *key )
{
    int slot;
    for (slot = ARRAY_SIZE - 2; (slot >= 0 && array[slot].price > key->price); slot--)
        array[slot + 1] = array[slot];
    array[slot+1] = *key;
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
    //uint8_t data[] = { 0x00, 0x00, 0x00, 0x00, 0x00 }; //0x12, 0x12, 0x12, 0x12, 0x12, 0x12}; //0x34, 0x56, 0xFF, 0xFF, 0x55, 0xF0};  //, 0x0F, 0x01, 0x02 };
    uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
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

#define NUM_SLOTS   4
    APV2Cost<uint8_t> cost_slot[NUM_SLOTS][n+1];
    cost_slot[0][0].price = 0;

    for( size_t i=0; i<n; ++i )
    {

        size_t np = i+1;
        for(uint8_t from_slot = 0; from_slot<NUM_SLOTS; ++from_slot )
        {
            // skip of no valid estimate for current slot
            if( cost_slot[from_slot][i].price == SIZE_MAX )
                break;

            APV2Cost<uint8_t> tmp = cost_slot[from_slot][i];
            uint32_t literal_cost = tmp.cost_literal(addr[i]);
            if( cost_slot[NUM_SLOTS-1][np].price >
                cost_slot[from_slot][i].price + literal_cost )
            {
                tmp.price = cost_slot[from_slot][i].price + literal_cost;
                tmp.link = i;
                tmp.slot = from_slot;
                // update statistics
                int slot;
                for (slot = NUM_SLOTS - 2; (slot >= 0 && cost_slot[slot][np].price > tmp.price); slot--)
                    cost_slot[slot + 1][np] = cost_slot[slot][np];
                printf("literal slot: %d\n", slot);
                cost_slot[slot+1][np] = tmp;
            }
        }

        int j = ISA[i];
        printf("=>%d %d\n", i, j);

        int lpsv = n;
        // forward scan for all matches
        for( int k=j+1; k<n; ++k )
        {
            int l = LCP[k];
            lpsv = std::min( l, lpsv );
//            printf("lpsv: %d\n", lpsv);
            if( lpsv < 1 )
                break;

            int m = SA[k];
            if( m > i )
                continue;

            size_t lenght = lpsv;
            size_t offset = i-m;
            np = i+lenght;
            printf("lenght: %d\noffset: %d\n", lenght, offset);
//                printf("%d <-> %d\n", old_np, np);

            for(uint8_t from_slot = 0; from_slot<NUM_SLOTS; ++from_slot )
            {
                // skip if no valid estimate for current slot
                if( cost_slot[from_slot][i].price == SIZE_MAX )
                    break;

                APV2Cost<uint8_t> tmp = cost_slot[from_slot][i];
                uint32_t match_cost = tmp.cost_match( lenght, offset );
                if( cost_slot[NUM_SLOTS-1][np].price >
                    cost_slot[from_slot][i].price + match_cost )
                {
                    tmp.price = cost_slot[from_slot][i].price + match_cost;
                    tmp.link = i;
                    tmp.slot = from_slot;
                    // update statistics, keep slots sorted by price
                    int slot;
                    for (slot = NUM_SLOTS - 2; (slot >= 0 && cost_slot[slot][np].price > tmp.price); slot--)
                        cost_slot[slot + 1][np] = cost_slot[slot][np];
                    printf("from_slot: %d slot: %d\n", from_slot, slot+1);
//                        printf("match slot: %d\n", slot);
                    cost_slot[slot+1][np] = tmp;
                }
                printf("forward (%4d|%4d)\n", i-m, lpsv);
            }
        }

        int lnsv = n;
        // backward scan for all matches
        for( int k=j-1; k>0; --k )
        {
            int l = LCP[k+1];
            lnsv = std::min( l, lnsv );
//            printf("lnsv: %d\n", lnsv);
            if( lnsv < 1 )
                break;

            int m = SA[k];
            if( m > i )
                continue;

            size_t lenght = lnsv;
            size_t offset = i-m;
            np = i+lenght;
#if 1
            for(uint8_t from_slot = 0; from_slot<NUM_SLOTS; ++from_slot )
            {
                // skip if no valid estimate for current slot
                if( cost_slot[from_slot][i].price == SIZE_MAX )
                    break;

                APV2Cost<uint8_t> tmp = cost_slot[from_slot][i];
                uint32_t match_cost = tmp.cost_match( lenght, offset );
                if( cost_slot[NUM_SLOTS-1][np].price >
                    cost_slot[from_slot][i].price + match_cost )
                {
                    tmp.price = cost_slot[from_slot][i].price + match_cost;
                    tmp.link = i;
                    tmp.slot = from_slot;
                    // update statistics, keep slots sorted by price
                    int slot;
                    for (slot = NUM_SLOTS - 2; (slot >= 0 && cost_slot[slot][np].price > tmp.price); slot--)
                        cost_slot[slot + 1][np] = cost_slot[slot][np];
                    printf("from_slot: %d slot: %d\n", from_slot, slot+1);
//                        printf("match slot: %d\n", slot);
                    cost_slot[slot+1][np] = tmp;
                }
                printf("backward (%4d|%4d)\n", i-m, lnsv);
//                    break;
            }
#endif
        }
    }

//    APV2Cost<uint8_t> *cost = cost_slot[0];
#if 0
    uint8_t slot = 0;
    for( int i=n; i>0;)
    {
        printf("%d> ", i );
        for( int j=0; j<NUM_SLOTS; ++j )
            printf("%1d -> %6.2f ", cost_slot[j][i].slot, Q8TOF(cost_slot[j][i].price));
        printf("\n");
        printf("%1d -> %6.2f\n", cost_slot[slot][i].slot, Q8TOF(cost_slot[slot][i].price));
        uint8_t old_slot = slot;
        slot = cost_slot[old_slot][i].slot;
        i = cost_slot[old_slot][i].link;
    }
    printf("last slot: %d\n", slot);
#endif

    {
        size_t current = n;
        uint8_t slot_current = 0;
        size_t next = -1;
        size_t slot_next = -1;
    while( current != -1 )
    {
//        printf("current: %d slot: %d\n", cost_slot[slot][i].link, slot);
        next = cost_slot[slot_current][current].link;
        slot_next = cost_slot[slot_current][current].slot;

        printf("%2d -> %2d|%2d ", current, cost_slot[slot_current][current].link, cost_slot[slot_current][current].slot);
        for( int j=0; j<NUM_SLOTS; ++j )
            printf("[%2d|%2d] ", cost_slot[j][current].link, cost_slot[j][current].slot );
        printf("\n");
        cost_slot[slot_current][current].print_encoding();
        current = next;
        slot_current = slot_next;
    }
    }
    // reverse links
    size_t prev = -1;
    size_t current = n;
    size_t next = -1;
    uint8_t slot_prev = -1;
    uint8_t slot_current = 0;
    uint8_t slot_next = -1;
    while (current != -1) {
        // Store next
        next = cost_slot[slot_current][current].link;
        slot_next = cost_slot[slot_current][current].slot;

        // Reverse current node's pointer
        cost_slot[slot_current][current].link = prev;
        cost_slot[slot_current][current].slot = slot_prev;

        // Move pointers one position ahead.
        prev = current;
        slot_prev = slot_current;
        current = next;
        slot_current = slot_next;
    }

    printf("after reversal %d\n", slot_prev);
    APV2Encode encoder(&buf[1024]);

    {
        size_t current = 1;
        uint8_t slot_current = 0;
        size_t next = 0;
        size_t slot_next = 0;
        while( current != -1 )
        {
//        printf("current: %d slot: %d\n", cost_slot[slot][i].link, slot);
            next = cost_slot[slot_current][current].link;
            slot_next = cost_slot[slot_current][current].slot;

            printf("%2d -> %2d|%2d ", current, cost_slot[slot_current][current].link, cost_slot[slot_current][current].slot);
            for( int j=0; j<NUM_SLOTS; ++j )
                printf("[%2d|%2d] ", cost_slot[j][current].link, cost_slot[j][current].slot );
            printf("\n");
            APV2Cost<uint8_t> *cost = &cost_slot[slot_current][current];

            printf("encode: %d, %d\n", cost->encode.lenght, cost->encode.offset);
            if( cost->encode.lenght == 0 )
                encoder.encode_literal( cost->encode.offset );
            else
                encoder.encode_match( cost->encode.lenght, cost->encode.offset );
            current = next;
            slot_current = slot_next;
        }
    }
    uint8_t *start = encoder.flush();

    for( uint8_t *ptr=start; ptr<&buf[1024]; ++ptr )
    {
        printf("%02x, \n", *ptr);
    }

    APV2 decoder(start);
    decoder.decompress( buf, 34 );
    // force equality
    for( int i=0; i<n; ++i )
    {
//        printf("%c", buf[i]);
        assert( buf[i]==addr[i]);
    }
    uint8_t slot = slot_prev;
    for( int i=1; i!=-1;)
    {
        printf("current: %d slot: %d\n", cost_slot[slot][i].link, slot);
        printf("%d> ", i );
        for( int j=0; j<NUM_SLOTS; ++j )
            printf("%1d -> %6.2f ", cost_slot[j][i].slot, Q8TOF(cost_slot[j][i].price));
        printf("\n");
        printf("%1d -> %6.2f\n", cost_slot[slot][i].slot, Q8TOF(cost_slot[slot][i].price));
        uint8_t old_slot = slot;
        slot = cost_slot[old_slot][i].slot;
        i = cost_slot[old_slot][i].link;
    }


    uint8_t probs[256];
    for( int i=0; i<256; ++i )
        probs[i] = 110;

    rABSEncoder<256, 32768, uint8_t> out(&buf[1024]);

    for( int j=0; j<sizeof(data); ++j )
    {
        out.encode_bit_tree<8>( data[j], probs );
    }
    start = out.flush();

    printf("encoded:\n");
    for( uint8_t *ptr=start; ptr<&buf[1024]; ++ptr )
    {
        printf("%02x, \n", *ptr);
    }

    for( int i=0; i<256; ++i )
        probs[i] = 110;

    rABSDecode<256, 32768, uint8_t> dec( start );


    printf("decoded:\n");
    for( int j=0; j<sizeof(data); j++ )
    {
        uint8_t value = dec.decode_bit_tree<8>( probs );
        printf("%02x\n", value);
    }

    printf("cost encode:\n");
    APV2Cost<uint8_t> cost;

    for( int i=0; i<256; ++i )
        probs[i] = 128;

    size_t total = 0;
    for( int j=0; j<sizeof(data); ++j )
    {
        total += cost.cost_bit_tree<8>( data[j], probs );
        printf("%f\n", Q8TOF(total));
    }
    printf("total: %f\n", Q8TOF(total));
    for( int i=0; i<256; ++i )
        probs[i] = 255; //110;



    munmap(addr, sb.st_size);
    close(fd);

    return EXIT_SUCCESS;
}


