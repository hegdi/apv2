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

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "libsais.h"
#include "matchfinder.h"

#define max(a,b) \
	({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
	({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

typedef struct {
	int32_t *SA;
	int32_t *ISA;
	int32_t *LCP;
	void *libsais_ctx;

	int32_t max_offset;
	int32_t min_match_length;
	int32_t max_match_length;
	int32_t block_size;

	int32_t *visited;
} mf_context_t;

void * matchfinder_create(int32_t max_block_size, int32_t max_offset, int32_t min_match_length, int32_t max_match_length) {
	mf_context_t *ctx = calloc(1, sizeof(mf_context_t));
	ctx->libsais_ctx = libsais_create_ctx();
    ctx->SA  = calloc(max_block_size, sizeof(int32_t));
    ctx->ISA = calloc(max_block_size, sizeof(int32_t));
    ctx->LCP = calloc(max_block_size+1, sizeof(int32_t));
    ctx->max_offset = max_offset;
    ctx->min_match_length = min_match_length;
    ctx->max_match_length = max_match_length;

    ctx->visited = calloc(max_offset+1, sizeof(int32_t));
    return (void*)ctx;
}

void matchfinder_destroy(void * mf) {
	mf_context_t *ctx = (mf_context_t*)mf;

	assert( ctx != NULL );
	free( ctx->SA );
	free( ctx->ISA );
	free( ctx->LCP );
	free( ctx->visited );
	libsais_free_ctx(ctx->libsais_ctx);
	free( ctx );
}


int32_t matchfinder_parse(void * mf, const uint8_t * block, int32_t block_size) {
	mf_context_t *ctx = (mf_context_t*)mf;

    int32_t *SA = ctx->SA;
    int32_t *LCP = ctx->LCP;
    int32_t *PLCP = calloc(block_size, sizeof(int32_t));
    int32_t *ISA = ctx->ISA;
    int32_t min_match_length = ctx->min_match_length;
    ctx->block_size = block_size;

	int32_t ret = libsais_ctx( ctx->libsais_ctx, block, SA, block_size, 0, NULL );
	if( ret < 0 ) {
		return -1;
	}
    ret = libsais_plcp( block, SA, PLCP, block_size );
	if( ret < 0 ) {
		return -1;
	}
    ret = libsais_lcp( PLCP, SA, LCP, block_size );
	if( ret < 0 ) {
		return -1;
	}

    free(PLCP);

    for( int32_t i=0; i<block_size; ++i ) {
        // clamp lcp to min match length here to avoid comparisons later on
        // and communicated matches which exceed max_match_lenght
        LCP[i] = (LCP[i]<min_match_length)?0:LCP[i];
        ISA[SA[i]] = i;
    }

    return 0;
}

uint32_t matchfinder_find_all_matches(void * mf, int32_t p, matchfinder_match_t * matches) {
	mf_context_t *ctx = (mf_context_t*)mf;

	matchfinder_match_t *matchptr = matches;
	int32_t *SA = ctx->SA;
	int32_t *ISA = ctx->ISA;
	int32_t *LCP = ctx->LCP;
	int32_t max_offset = ctx->max_offset;
	int32_t block_size = ctx->block_size;
	int32_t *visited = ctx->visited;

    int32_t j = ISA[p];
    int32_t block_remaining = block_size-p;
    int32_t lpsv = block_remaining;
    int32_t lnsv = block_remaining;
    memset(visited, 0, sizeof(int32_t)*(max_offset+1));
    for(int32_t next=j+1, prev=j-1; ((next<block_size) && (LCP[next]>0)) || ((prev>0) && (LCP[prev+1]>0));) {
        lnsv = min(LCP[next], lnsv);
        lpsv = min(LCP[prev+1], lpsv);

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

        if( m > p ) {
            continue;
        }
        // limits the number of generated matches to be below max_offset
        int32_t offset = p-m;
        if( offset > max_offset ) {
            continue;
        }
        visited[offset] = length;

        // sanitize matches in regions of low entropy
        if( visited[offset-1] == length ) {
            continue;
        }
//        if( (offset+1<max_offset) && (visited[offset+1] == length) ) {
//            continue;
//        }
        if( (offset+1<max_offset) && (visited[offset+1] == length+1) ) {
            continue;
        }

        matches->length = length;
		matches->offset = offset;
		++matches;
    }
    return matches - matchptr;
}

