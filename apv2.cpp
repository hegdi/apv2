#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <cmath>
#include <vector>
#include <string>
#include <filesystem>

#include "sais.h"

static inline int log2i(int x) {
    assert(x > 0);

    return sizeof(int) * 8 - __builtin_clz(x) - 1;
}

#define Q8TOF(x)    ((x)/256.f)
#define Q12TOF(x)    ((x)/4096.f)
#define FTOQ8(x)    ((int)((x)*256.f+0.5f))

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


#define APV2_INITIAL_PROP   FTOQ8(0.43f)
class APV2 : private rABSDecode<256,32768,uint8_t>
{
    typedef uint8_t prob_t;
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
        uint8_t *ptr = dest;
        for(; ptr<dest+size;)
        {
            uint8_t length = decode_bit_tree<8>( lengthModel );
//            printf("%5d length: %3d ", ptr-dest, length);
            if( length == 0 )
            {
                uint8_t lit = decode_bit_tree<8>( literalModel );
                *ptr++ = lit;
//                printf("lit: %#04x\n", lit);
                continue;
            }
            uint16_t offset = decode_bit_tree<12>( offsetModel );
//            printf("offset: %4d\n", offset);
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

uint16_t log2table[256] = { 0 };

void initQ8log2()
{
    for( int i = 1; i<256; ++i )
    {
        log2table[i] = -log2( i/256.f )*4096.f+0.5f;
    }
}
uint16_t Q8log2( uint8_t x )
{
    return log2table[x];
}

class APV2Encode : public rABSEncoder<256,32768,uint8_t>
{
    typedef uint8_t prob_t;
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
    unsigned lastOffset;

};

class APV2Cost
{
    typedef uint8_t prob_t;
public:
    APV2Cost() : lastOffset( 0 )
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
        encode = { 0, 0 };
    }

    int cost_bit( unsigned sym, prob_t *probability )
    {
        prob_t prob = *probability;
        *probability = adapt<256, 4>( sym, prob );

        // prob is for symbol 0
        if( !sym )
            prob = 256 - prob; // for symbol 1

        return Q8log2( prob );
    }

    template<unsigned NumBits>
    uint32_t cost_bit_tree(unsigned symbol, prob_t *probs)
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

    struct {
        uint8_t lenght;
        uint16_t offset;
    } encode;
    unsigned lastOffset;

    static const int maxMatch = 255;
    static const int minMatch = 1;
    static const int maxOffset = 4095;
    static const int minOffset = 1;
private:

    prob_t literalModel[256];
    prob_t lengthModel[256];
    prob_t offsetModel[4096];

};

template<typename C, unsigned NUM_SLOTS>
class Forward_arrivals_parse
{
    class Cost : public C {
    public:
        Cost() : price(SIZE_MAX), link( -1 ), slot( -1 ) {}
        size_t price;
        size_t link;
        int_fast8_t slot;
    };
    Cost **costs;

public:
    Forward_arrivals_parse( uint8_t *ptr, size_t n ) : data( ptr ), dataSize( n )
    {
        costs = new Cost*[n+1];
        for (size_t i = 0; i < n+1; ++i)
            costs[i] = new Cost[NUM_SLOTS];

        costs[0][0].price = 0;

        SA = new int[n+1];
        LCP = new int[n];
        ISA = new int[n];

        sais(data, SA, LCP, n );

        for( size_t i=0; i<n; ++i )
        {
            ISA[SA[i]] = i;
        }
    }

    ~Forward_arrivals_parse()
    {
        for( size_t i = 0; i<dataSize+1; ++i )
            delete [] costs[i];
        delete [] costs;

        delete [] SA;
        delete [] LCP;
        delete [] ISA;
    }

    void find_slot( Cost array[], size_t s, Cost *key )
    {
        if( s == 0 ) // expand search from shortest arrival
        for(unsigned slot = 0; slot<NUM_SLOTS; ++slot)
        {
            if( array[slot].price > key->price )
            {
                array[slot] = *key;
                return;
            }
        }
        else // collapse search from all others
        {
            for(int slot = s; slot>=0; slot--)
            {
                if(array[slot].price > key->price )
                {
                    array[slot] = *key;
                    return;
                }
            }
        }
    }

    void forward_arrival_literal( size_t p )
    {
        auto np = p+1;
        for(unsigned from_slot = 0; from_slot<NUM_SLOTS; ++from_slot )
        {
            // skip of no valid estimate for current slot
            if( costs[p][from_slot].price == SIZE_MAX )
                break;

            auto tmp = costs[p][from_slot];
            auto literal_cost = tmp.cost_literal(data[p]);
            tmp.price += literal_cost;
            tmp.link = p;
            tmp.slot = from_slot;

            find_slot( costs[np], from_slot, &tmp );
        }
    }

    void forward_arrival_match( size_t p, size_t length, size_t offset )
    {
        // match to far back
        if( offset > Cost::maxOffset )
            return;

        auto np = p+length;

        for(unsigned from_slot = 0; from_slot<NUM_SLOTS; ++from_slot )
        {
            // skip if no valid estimate for current slot
            if( costs[p][from_slot].price == SIZE_MAX )
                break;

            auto tmp = costs[p][from_slot];
            auto match_cost = tmp.cost_match( length, offset );
            tmp.price += match_cost;
            tmp.link = p;
            tmp.slot = from_slot;

            find_slot( costs[np], from_slot, &tmp );
        }

    }

    size_t parse( void(*progress)(size_t p, size_t max)=NULL )
    {
        for( size_t p=0; p<dataSize; ++p )
        {
            // progress tracking if available
            if( progress != NULL )
                progress( p, dataSize );

            forward_arrival_literal(p);

            auto j = ISA[p];

            auto lpsv = std::min<size_t>(Cost::maxMatch, dataSize-p);
            auto lnsv = std::min<size_t>(Cost::maxMatch, dataSize-p);
            for(size_t next=j+1, prev=j-1; (lpsv >= Cost::minMatch) || (lnsv >= Cost::minMatch);)
            {
                lpsv = (next>dataSize-1)?0:std::min<size_t>( LCP[next], lpsv );
                lnsv = (prev<1)?0:std::min<size_t>( LCP[prev+1], lnsv );

                if( (lpsv > 0) && (lpsv >= lnsv) )
                {
                    size_t m = SA[next];
                    if( m < p )
                    {
                        auto length = lpsv;
                        size_t offset = p-m;
                        forward_arrival_match( p, length, offset );
                    }
                    if( next < dataSize )
                        ++next;
                }
                if( (lnsv>0) && (lnsv >= lpsv) )
                {
                    size_t m = SA[prev];
                    if( m < p )
                    {
                        auto length = lnsv;
                        size_t offset = p-m;
                        forward_arrival_match( p, length, offset );
                    }
                    if( prev > 0 )
                        --prev;
                }
            }
        }

        size_t price = reverse();
#if DEBUG_PARSE
        size_t current = 1;
        int slot_current = 0;
        size_t lastOffset = 0;
        for( int p=1; p<=dataSize; ++p )
        {
            printf("%8d: ", p );
            for(auto from_slot = 0; from_slot<NUM_SLOTS; ++from_slot )
            {
                Cost *cost = &costs[p][from_slot];
                // skip of no valid estimate for current slot
                if( cost->price == SIZE_MAX )
                    break;

                int lenght = cost->encode.lenght;
                int offset = cost->encode.offset;

                char type = ' ';
                if( lenght == 0 )
                {
//                    offset = 0;
                }
                if( cost->encode.lenght > 0 )
                    type = 'M';
                if(( cost->encode.lenght > 0 ) && (cost->encode.offset == 0 ))
                {
                    type = 'L';
                    offset = cost->lastOffset;
                }
                if( (current == p) && (from_slot == slot_current) )
                {
                    printf("[%2d%c+%3d|%-2d] %d", lenght, type, offset,
                            cost->slot, cost->price);//Q8TOF( cost->price ));

                    current = cost->link;
                    slot_current = cost->slot;
                    lastOffset = cost->lastOffset;
                }
                else
                printf(" %2d%c+%3d|%-2d  %d", lenght, type, offset,
                        cost->slot, cost->price);//Q8TOF( cost->price ));

            }
            printf("\n");
        }
#endif
        return price;
    }

    size_t reverse()
    {
        // find slot with minimum price
        uint8_t slot_min = 0;
        for(unsigned i=0; i<NUM_SLOTS; ++i)
            if( costs[dataSize][i].price < costs[dataSize][slot_min].price )
                slot_min = i;

        // reverse links and slots
        size_t prev = -1;
        size_t current = dataSize;
        size_t next = -1;
        uint8_t slot_prev = -1;
        uint8_t slot_current = slot_min;
        uint8_t slot_next = -1;
        while (current != SIZE_MAX) {
            // Store next
            next = costs[current][slot_current].link;
            slot_next = costs[current][slot_current].slot;

            // Reverse current node's pointer
            costs[current][slot_current].link = prev;
            costs[current][slot_current].slot = slot_prev;

            // Move pointers one position ahead.
            prev = current;
            slot_prev = slot_current;
            current = next;
            slot_current = slot_next;
        }
        return costs[dataSize][slot_min].price;
    }

    template<typename T>
    class iterator {
    public:
      iterator(T *c, size_t p, int_fast8_t slot): costs(c->costs), current(p), slot_current(slot) {}
      iterator operator++()
      {
          auto next = costs[current][slot_current].link;
          auto slot_next = costs[current][slot_current].slot;

          current = next;
          slot_current = slot_next;
          return *this;
      }
      bool operator!=(const iterator & other) const { return (current != other.current) || (slot_current != other.slot_current); }
      const typename T::Cost& operator*() const { return costs[current][slot_current]; }
    private:
      typename T::Cost **costs;
      size_t current;
      int_fast8_t slot_current;
    };

    auto begin() { return iterator( this, 1, 0 ); }
    auto end() { return iterator( this, -1, -1 ); }

private:

    uint8_t *data;
    size_t dataSize;

    int* SA;
    int* LCP;
    int* ISA;

};

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

void dump_to_file( const char *file, void* ptr, size_t size )
{
    FILE *fd = fopen( file, "wb" );
    if(fd == NULL )
    {
        perror("can't open file");
        return;
    }
    fwrite( ptr, size, 1, fd );
    fclose( fd );
}

#include <signal.h>
#include <sys/ioctl.h>
unsigned COLS = 0;

void adjust( int signum )
{
    (void)signum;
    struct winsize sz;
    ioctl( 0, TIOCGWINSZ, &sz );
    COLS = sz.ws_col;
}

void print_progress(size_t count, size_t max)
{
    size_t bar_width = COLS-10;
    bar_width = (bar_width > 100)?100:bar_width;
#define PERCENTAGE(V, T) (V*100/T)
    static char bar[100+1] = "";
    size_t i;
    for (i=0; i < bar_width; ++i)
    {
        bar[i] = i < PERCENTAGE(count, max) ? '=' : ' ';
    }
    bar[i] = 0;

    printf("\e[2K\r|%s| %3.0f%%", bar, count*100.f/max);
    if( count == max-1 )
        printf("\n");
    fflush(stdout);
}


int main( int argc, char *args[] )
{
    if( argc < 2 )
    {
        printf("%s <filename>\n", args[0]);
        return EXIT_SUCCESS;
    }

    adjust(0);
    (void) signal(SIGWINCH, adjust);    /* arrange interrupts to resize */

    std::string filename = std::filesystem::path(args[1]).stem();
    filename += ".apv2";

    printf("out file: %s\n", filename.c_str());

    initQ8log2();

    struct stat sb;
    int fd = open(args[1], O_RDONLY);
    if (fd < 0)
    {
        perror("can't open file");
        return EXIT_FAILURE;
    }

    if (fstat(fd, &sb) < 0)           /* To obtain file size */
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

    Forward_arrivals_parse<APV2Cost, 16> parser(addr, n);
    size_t compressedBits = parser.parse(print_progress);
    size_t compressedBytes = compressedBits/(8*4096)+4;
    uint8_t *outBuf = new uint8_t[compressedBytes];
    APV2Encode encode(&outBuf[compressedBytes]);
    printf("compressed size: %f\n", Q12TOF(compressedBits));
    for( auto& i : parser )
    {
//        printf("length: %3d offset: %4d\n", i.encode.lenght, i.encode.offset);
        if( i.encode.lenght == 0 )
            encode.encode_literal( i.encode.offset );
        else
            encode.encode_match( i.encode.lenght, i.encode.offset );
    }
    uint8_t *src = encode.flush();
    size_t fileSize = &outBuf[compressedBytes]-src;
    printf("compressed file size: %zu\n", fileSize );
    dump_to_file( filename.c_str(), src, fileSize);

    printf("verify... ");
    uint8_t *verifyBuf = new uint8_t[n];
    APV2 decode(src);
    decode.decompress( verifyBuf, n );
    // force equality
    for( size_t i=0; i<n; ++i )
    {
//        printf("%c", buf[i]);
        assert( verifyBuf[i]==addr[i]);
    }
    printf("succeeded!\n");
    delete [] verifyBuf;
    delete [] outBuf;

    munmap(addr, sb.st_size);
    close(fd);

    return EXIT_SUCCESS;
}


