/*
 * matchfinder.c for apv2
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

#include <stdlib.h>

#include "libsais.h"
#include "matchfinder.h"

static inline int32_t min32(int32_t a, int32_t b) {
    return a < b ? a : b;
}

typedef struct {
    int32_t *SA;
    int32_t *ISA;
    int32_t *LCP;
    void *libsais_ctx;

    int32_t max_offset;
    int32_t min_match_length;
    int32_t max_match_length;
    int32_t block_size;
} mf_context_t;

void * matchfinder_create(int32_t max_block_size, int32_t max_offset, int32_t min_match_length, int32_t max_match_length) {
    if( max_block_size <= 0 || max_offset < 0 ) {
        return NULL;
    }
    mf_context_t *ctx = calloc(1, sizeof(mf_context_t));
    if( ctx == NULL ) {
        return NULL;
    }
    ctx->libsais_ctx = libsais_create_ctx();
    ctx->SA  = calloc(max_block_size, sizeof(int32_t));
    ctx->ISA = calloc(max_block_size, sizeof(int32_t));
    /* one slot beyond the block: the match walk reads the LCP entry of a
     * side once it is exhausted and relies on that entry holding 0 */
    ctx->LCP = calloc(max_block_size+1, sizeof(int32_t));
    if( ctx->libsais_ctx == NULL || ctx->SA == NULL
        || ctx->ISA == NULL || ctx->LCP == NULL ) {
        matchfinder_destroy(ctx);
        return NULL;
    }
    ctx->max_offset = max_offset;
    ctx->min_match_length = min_match_length;
    ctx->max_match_length = max_match_length;

    return (void*)ctx;
}

void matchfinder_destroy(void * mf) {
    mf_context_t *ctx = (mf_context_t*)mf;

    if( ctx == NULL ) {
        return;
    }
    free( ctx->SA );
    free( ctx->ISA );
    free( ctx->LCP );
    if( ctx->libsais_ctx != NULL ) {
        libsais_free_ctx(ctx->libsais_ctx);
    }
    free( ctx );
}


int32_t matchfinder_parse(void * mf, const uint8_t * block, int32_t block_size) {
    mf_context_t *ctx = (mf_context_t*)mf;

    int32_t *SA = ctx->SA;
    int32_t *LCP = ctx->LCP;
    int32_t *PLCP = calloc(block_size, sizeof(int32_t));
    int32_t *ISA = ctx->ISA;
    int32_t min_match_length = ctx->min_match_length;
    int32_t max_match_length = ctx->max_match_length;
    if( PLCP == NULL ) {
        return -1;
    }
    ctx->block_size = block_size;

    int32_t ret = libsais_ctx( ctx->libsais_ctx, block, SA, block_size, 0, NULL );
    if( ret < 0 ) {
        free(PLCP);
        return -1;
    }
    ret = libsais_plcp( block, SA, PLCP, block_size );
    if( ret < 0 ) {
        free(PLCP);
        return -1;
    }
    ret = libsais_lcp( PLCP, SA, LCP, block_size );
    if( ret < 0 ) {
        free(PLCP);
        return -1;
    }

    free(PLCP);

    for( int32_t i=0; i<block_size; ++i ) {
        // clamp the lcp to the reportable window [min, max]: entries below
        // min are zeroed so the match walk, which stops at LCP entries of 0,
        // never visits those suffixes. Capping entries at max keeps the
        // reported lengths true but never above the format's limit
        int32_t l = LCP[i];
        LCP[i] = (l<min_match_length)?0:((l>max_match_length)?max_match_length:l);
        ISA[SA[i]] = i;
    }

    return 0;
}

/*
 * Reports every distinct offset in the window with the length of the shared
 * prefix, clamped to [min_match_length, max_match_length] - the LCP interval
 * minimum at that suffix's rank is the true lcp of the two suffixes, so
 * nothing is dropped or approximated. Choosing which candidates are worth
 * parsing is the parser's job: only it knows what a match costs in the
 * current model state. One entry per offset holds naturally (offset
 * identifies the source position), so the result never exceeds max_offset
 * entries.
 */
uint32_t matchfinder_find_all_matches(void * mf, int32_t p, matchfinder_match_t * matches) {
    mf_context_t *ctx = (mf_context_t*)mf;

    matchfinder_match_t *matchptr = matches;
    int32_t *SA = ctx->SA;
    int32_t *ISA = ctx->ISA;
    int32_t *LCP = ctx->LCP;
    int32_t max_offset = ctx->max_offset;
    int32_t block_size = ctx->block_size;

    int32_t j = ISA[p];
    int32_t block_remaining = block_size-p;
    int32_t lpsv = block_remaining;
    int32_t lnsv = block_remaining;
    for(int32_t next=j+1, prev=j-1; ((next<block_size) && (LCP[next]>0)) || ((prev>=0) && (LCP[prev+1]>0));) {
        // the exhausted side reads the zero kept beside the used range
        // and stays out of the race afterwards
        lnsv = min32(LCP[next], lnsv);
        lpsv = min32(LCP[prev+1], lpsv);

        int32_t m, length;
        if( lnsv > lpsv ) {
            m = SA[next];
            length = lnsv;
            ++next;
        } else {
            m = SA[prev];
            length = lpsv;
            --prev;
        }

        if( m >= p ) {
            continue;
        }
        // limits the number of generated matches to be below max_offset
        int32_t offset = p-m;
        if( offset > max_offset ) {
            continue;
        }

        matches->length = length;
        matches->offset = offset;
        ++matches;
    }
    return matches - matchptr;
}

