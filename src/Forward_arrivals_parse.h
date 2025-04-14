/*
 * Forward_arrival_parse.h for apv2
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

#ifndef FORWARD_ARRIVALS_PARSE_H_
#define FORWARD_ARRIVALS_PARSE_H_

#include <boost/intrusive/list.hpp>
#include <cstdio>
#include <string>
#include <vector>

#ifdef USE_ESA_MATCHFINDER
#include <esa_matchfinder.h>
#else
#include "matchfinder.h"
#endif

#define USE_ESA_MATCHFINDERx
#define DEBUG_PARSEx

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

using namespace boost::intrusive;

template<typename C>
class Forward_arrivals_parse {

    class Cost : public C {
    public:
        Cost() : price(SIZE_MAX), ref(0), slot( NULL ), link( NULL ), score( 0 ) {
        }
        ~Cost() {
        }
        size_t price;
        unsigned int ref;
        Cost *slot;
        friend bool operator<(const Cost& a, const Cost& b) {
            if( a.price < b.price ) return true;
            if( a.price > b.price ) return false;
            return a.score < b.score;
        }
        friend bool operator>(const Cost& a, const Cost& b) {
            return b < a;
        }

        typedef list_member_hook< link_mode< auto_unlink>> list_node_t;
        list_node_t list_node;
        typedef list< Cost
                    , member_hook< Cost, list_node_t, &Cost::list_node>
                    , constant_time_size< false>
                    > List;
        List *link;
        size_t score;
#ifdef DEBUG_PARSE
        unsigned from;
#endif
    };
public:
    static void log_list( size_t p, typename Cost::List &list ) {
        std::string filename = std::to_string(p) + ".csv";
        FILE *l = fopen(filename.c_str(), "w");
        fprintf(l, "price\n");
        for( auto& slot : list ) {
            fprintf(l, "%e\n", slot.price/QFACTOR);
        }
        fclose( l);
    }

    Forward_arrivals_parse( uint8_t *ptr, size_t n, size_t edge_pool_size ) : chunkSize( edge_pool_size ), data( ptr ), dataSize( n ) {
        allocLevel = 0;
        poolSize = 0;
        lowLevel = chunkSize*0.01f;
        allocPool();

        costs = new typename Cost::List[n+1];
        nbrEdges = new size_t[n+1];

#ifdef USE_ESA_MATCHFINDER
        mf = esa_matchfinder_create(dataSize, Cost::minMatch, Cost::maxMatch);
#else
        mf = matchfinder_create(dataSize, /* max_offset */ Cost::maxOffset, /*min_match_length*/ Cost::minMatch, /*max_match_length*/ Cost::maxMatch);
#endif
        assert( mf != NULL );
#ifdef USE_ESA_MATCHFINDER
        int32_t ret = esa_matchfinder_parse(mf, data, dataSize);
#else
        int32_t ret = matchfinder_parse(mf, data, dataSize);
        (void)(ret);
        assert ( ret == 0 );
#endif
    }

    ~Forward_arrivals_parse() {
        delete[] costs;
        delete[] nbrEdges;
#ifdef USE_ESA_MATCHFINDER
        esa_matchfinder_destroy(mf);
#else
        matchfinder_destroy(mf);
#endif
        for( Cost *pool : edgePool ) {
            delete[] pool;
        }
    }

    void find_slot( typename Cost::List &list, Cost &key, size_t np ) {
        Cost &current_worst = list.back();
        if( (nbrEdges[np] > (unsigned)Cost::maxSlots) && (key.price > current_worst.price) ) {
            return;
        }

        if (unlikely(list.empty())) {
            Cost &newItem = allocEdge();
            newItem = key;
            newItem.slot->ref++;
            list.push_back(newItem);
            nbrEdges[np]++;
            return;
        }

        typename Cost::List::iterator insertion_point = list.end();
        // is there someone already better than us?
        bool exists = false;

        int key_prev_opcode = key.slot->encode.op;
        unsigned nbr_slots = 0;
        for( Cost &item : list ) {
            int item_prev_opcode = item.slot->encode.op;
            // if better exists already, don't bother
            if((item.price <= key.price)
               && (key_prev_opcode == item_prev_opcode)
               && (item.score <= key.score)
               && (item.isRepSetEqual(key)))
            {
                exists = true;
                break;
            }

            // note insertion location
            if( item > key ) {
                insertion_point = list.iterator_to(item);
                break;
            }

            if( nbr_slots > (unsigned)Cost::maxSlots ) {
                return;
            }
            nbr_slots++;
        }
        if(!exists) {
            Cost &newItem = allocEdge();
            newItem = key;

            newItem.slot->ref++;
            list.insert( insertion_point, newItem );
            nbrEdges[np]++;
        }
#ifndef DEBUG_PARSE
        return;
#else
        // verify
        for( auto i = list.begin(); i!=list.end(); ) {
            // terminate if we have only one element left
            Cost &itemA = *i++;
            if( i == list.end() ) {
                break;
            }
            Cost &itemB = *i;
            assert( itemA.price <= itemB.price );
            if (itemA.price > itemB.price) {
                printf("edges are out of order!\n");
                exit(EXIT_FAILURE);
            }
        }
#endif
    }

    void forward_arrival_literal( size_t p, Cost &from_slot ) {
        auto np = p+1;

        static Cost tmp;
        tmp = from_slot;

        tmp.ref = 0;
        auto literal_cost = tmp.cost_literal(data[p]);
        tmp.price += literal_cost;
        tmp.link = &costs[p];
        tmp.slot = &from_slot;
        tmp.score += Cost::scoreLiteral;

        find_slot( costs[np], tmp, np );
    }

    void forward_arrival_rep_match( size_t p, Cost &from_slot, size_t length, int slot ) {
        // match not in slot
        assert( slot < Cost::maxRepSlots );
        assert( length <= Cost::maxMatch );
        auto np = p+length;

        static Cost tmp;
        tmp = from_slot;
        tmp.ref = 0;
        auto match_cost = tmp.cost_rep_match( length, slot );
        tmp.price += match_cost;
        tmp.link = &costs[p];
        tmp.slot = &from_slot;
        tmp.score += Cost::scoreRepMatch;

        find_slot( costs[np], tmp, np );
    }

    void forward_arrival_match( size_t p, Cost &from_slot, size_t length, size_t offset ) {
        // match to far back
        assert( offset <= Cost::maxOffset );
        assert( length <= Cost::maxMatch );

        auto np = p+length;

        static Cost tmp;
        tmp = from_slot;
        tmp.ref = 0;
        auto match_cost = tmp.cost_match( length, offset );
        tmp.price += match_cost;
        tmp.link = &costs[p];
        tmp.slot = &from_slot;
        tmp.score += Cost::scoreMatch;

        find_slot( costs[np], tmp, np );
    }

    typedef void(*progress_callback_t)(size_t p, size_t max, size_t poolLevel, size_t poolSize);
    size_t parse( progress_callback_t progress=NULL ) {
        Cost &first = allocEdge();
        first.link = NULL;
        first.slot = NULL;
        first.price = 0;
        for( size_t i=0; i<dataSize; ++i ) {
            nbrEdges[i] = 0;
        }
        costs[0].push_back( first );

#ifdef USE_ESA_MATCHFINDER
        ESA_MATCHFINDER_MATCH matches[ESA_MATCHFINDER_MAX_MATCH_LENGTH];
        esa_matchfinder_rewind(mf, /*position*/ 0);
#else
        matchfinder_match_t matches[Cost::maxOffset];
#endif

        for( size_t p=0; p<dataSize; ++p ) {
            // progress tracking if available
            if( progress != NULL )
                progress( p+1, dataSize, sizeof(Cost)*allocLevel, sizeof(Cost)*poolSize );
#ifdef USE_ESA_MATCHFINDER
            ESA_MATCHFINDER_MATCH *matchptr = esa_matchfinder_find_all_matches(mf, matches);
            uint32_t total_matches = matchptr-matches;
#else
            uint32_t total_matches = matchfinder_find_all_matches(mf, p, matches);
#endif

            if( costs[p].empty() ) {
                continue;
            }

            size_t block_remaining = std::min<size_t>(Cost::maxMatch, dataSize-p);
            for (Cost &from_slot : costs[p]) {
                for (uint32_t j = 0; j < total_matches; ++j) {
#ifdef USE_ESA_MATCHFINDER
                    ESA_MATCHFINDER_MATCH *match = &matches[j];
                    int32_t offset = p-match->offset;
                    if( offset > Cost::maxOffset ) {
                        continue;
                    }
#else
                    matchfinder_match_t *match = &matches[j];
                    int32_t offset = match->offset;
#endif
                    int32_t length = std::min<int32_t>(match->length, block_remaining);
                    int slot = from_slot.isOffsetInSet( offset );
                    size_t minMatch = Cost::minMatch;
                    if( match->length > Cost::maxMatch ) {
                        length = Cost::maxMatch;
                    }
                    // test for repeat matches
                    if( slot > -1 ) {
                        for (size_t i = minMatch; i <= (size_t)length; ++i) {
                            forward_arrival_rep_match(p, from_slot, i, slot);
                            forward_arrival_match(p, from_slot, i, offset);
                        }
                        continue;
                    }
                    // test for normal matches
                    for (size_t i = minMatch; i <= (size_t)length; ++i) {
                        forward_arrival_match(p, from_slot, i, offset);
                    }
                }
                // and finally the literals
                forward_arrival_literal(p, from_slot);
            }

            if( allocLevel < lowLevel ) {
                purgePosition(p);
                if( allocLevel < lowLevel ) {
                    // didn't help so increase the pool
                    allocPool();
                }
            }
        }
        size_t price = reverse();

#ifdef DEBUG_PARSE
        typename Cost::List *current = &costs[1];
        Cost *slot_current = &current->front();
        unsigned slot_idx = 0;
        for( size_t p=1; p<=dataSize; ++p ) {
            printf("%8zu: ", p );
            if( costs[p].empty() ) {
                printf("null\n");
                continue;
            }
            unsigned idx = 0;
            for( Cost &cost : costs[p] ) {
                cost.from = idx++;

                int length = cost.encode.length;
                int offset = cost.encode.offset;

                char type = ' ';
                switch( cost.encode.op ) {
                case Cost::encode_t::LITERAL:
                    break;
                case Cost::encode_t::MATCH:
                    type = 'M';
                    break;
                case Cost::encode_t::REPEAT:
                    type = 'L';
                    offset = cost.getRep(cost.encode.offset);
                    break;
                default:
                    break;
                }
                if( (current == &costs[p]) && (&cost == slot_current) ) {
                    printf("\e[30;102m[%3d%c+%4d|%-2u]\e[0m", length, type, offset,
                            slot_idx);

                    current = cost.link;
                    slot_current = cost.slot;
                    slot_idx = cost.from;
                }
                else {
                    printf(" %3d%c+%4d|%-2u ", length, type, offset,
                            cost.slot->from);
                }
            }
            printf("\n");
        }
#endif
        purgePosition(dataSize);
        return price;
    }

    size_t reverse() {
        Cost &slot_min = costs[dataSize].front();

        // reverse links and slots
        typename Cost::List *prev = NULL;
        typename Cost::List *current = &costs[dataSize];
        typename Cost::List *next = NULL;
        Cost *slot_prev = NULL;
        Cost *slot_current = &slot_min;
        Cost *slot_next = NULL;
        while (slot_current->slot != NULL) {
            // Store next
            next = slot_current->link;
            slot_next = slot_current->slot;

            // Reverse current node's pointer
            slot_current->link = prev;
            slot_current->slot = slot_prev;

            // Move pointers one position ahead.
            prev = current;
            slot_prev = slot_current;
            current = next;
            slot_current = slot_next;
        }
        return slot_min.price;
    }

    template<typename T>
    class iterator {
    public:
      iterator(T *c, typename Cost::List *p, typename T::Cost *slot): costs(c->costs), current(p), slot_current(slot) {
      }
      iterator operator++() {
          auto next = slot_current->link;
          auto slot_next = slot_current->slot;

          current = next;
          slot_current = slot_next;
          return *this;
      }
      bool operator!=(const iterator & other) const {
          return (current != other.current) || (slot_current != other.slot_current);
      }
      const typename T::Cost& operator*() const {
          return *slot_current;
      }
    private:
      typename Cost::List *costs;
      typename Cost::List *current;
      typename T::Cost *slot_current;
    };

    auto begin() {
        return iterator( this, &costs[1], &costs[1].front() );
    }
    auto end() {
        return iterator( this, NULL, NULL );
    }

private:
    void allocPool() {
        Cost *pool = new Cost[chunkSize];
        edgePool.push_back( pool );
        for( size_t i=0; i<chunkSize; ++i ) {
            free_list.push_back(pool[i]);
        }
        allocLevel += chunkSize;
        poolSize += chunkSize;
        chunkSize *= 2;
    }

    Cost &allocEdge() {
        if( unlikely(free_list.empty()) ) {
            allocPool();
        }

        Cost &item = free_list.front();
        free_list.pop_front();
        allocLevel--;
        return item;
    }

    void freeEdge(Cost &edge) {
        edge.ref = 0; edge.link = NULL; edge.slot = NULL;
        free_list.push_back( edge );
        allocLevel++;
    }

    void purgePosition( size_t p ) {
        if( p < 1 ) {
            return;
        }
        for( size_t i=p-1; i>0; --i ) {
            typename Cost::List &list = costs[i];

            for( auto j = list.begin(); j!=list.end(); ) {
                Cost *slot = &*j++;
                while( (slot->slot != NULL) && (slot->ref == 0)) {
                    slot->slot->ref--;
                    Cost *slot_next = slot->slot;
                    slot->list_node.unlink();
                    freeEdge( *slot );
                    slot = slot_next;
                }
            }
        }
    }

    size_t allocLevel;
    size_t lowLevel;
    size_t poolSize;
    size_t chunkSize;
    std::vector<Cost *> edgePool;
    typename Cost::List free_list;

    typename Cost::List *costs;
    size_t *nbrEdges;

    void *mf;
    const uint8_t *data;
    size_t dataSize;
};

#endif /* FORWARD_ARRIVALS_PARSE_H_ */
