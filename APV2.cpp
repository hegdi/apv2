/*
 * APV2.cpp for apv2
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

#include <cmath>
#include "APV2.h"

uint32_t log2table[256] = {
         0, 524288, 458752, 420416, 393216, 372118, 354880, 340305,
    327680, 316544, 306582, 297571, 289344, 281776, 274769, 268246,
    262144, 256412, 251008, 245896, 241046, 236433, 232035, 227832,
    223808, 219948, 216240, 212672, 209233, 205915, 202710, 199610,
    196608, 193699, 190876, 188135, 185472, 182881, 180360, 177904,
    175510, 173175, 170897, 168672, 166499, 164374, 162296, 160262,
    158272, 156322, 154412, 152540, 150704, 148903, 147136, 145401,
    143697, 142024, 140379, 138763, 137174, 135611, 134074, 132561,
    131072, 129606, 128163, 126741, 125340, 123960, 122599, 121258,
    119936, 118632, 117345, 116076, 114824, 113588, 112368, 111163,
    109974, 108800, 107639, 106493, 105361, 104242, 103136, 102043,
    100963,  99894,  98838,  97793,  96760,  95738,  94726,  93726,
     92736,  91756,  90786,  89826,  88876,  87935,  87004,  86082,
     85168,  84263,  83367,  82479,  81600,  80728,  79865,  79009,
     78161,  77321,  76488,  75662,  74843,  74032,  73227,  72429,
     71638,  70853,  70075,  69303,  68538,  67778,  67025,  66278,
     65536,  64800,  64070,  63346,  62627,  61913,  61205,  60502,
     59804,  59111,  58424,  57741,  57063,  56390,  55722,  55059,
     54400,  53745,  53096,  52450,  51809,  51173,  50540,  49912,
     49288,  48668,  48052,  47440,  46832,  46228,  45627,  45031,
     44438,  43849,  43264,  42682,  42103,  41529,  40957,  40390,
     39825,  39264,  38706,  38152,  37600,  37052,  36507,  35965,
     35427,  34891,  34358,  33829,  33302,  32778,  32257,  31739,
     31224,  30711,  30202,  29695,  29190,  28689,  28190,  27694,
     27200,  26709,  26220,  25734,  25250,  24769,  24290,  23814,
     23340,  22869,  22399,  21933,  21468,  21006,  20546,  20088,
     19632,  19179,  18727,  18278,  17831,  17386,  16943,  16502,
     16064,  15627,  15192,  14760,  14329,  13900,  13473,  13048,
     12625,  12204,  11785,  11367,  10952,  10538,  10126,   9716,
      9307,   8901,   8496,   8093,   7691,   7291,   6893,   6497,
      6102,   5709,   5317,   4927,   4539,   4152,   3767,   3384,
      3002,   2621,   2242,   1865,   1489,   1115,    742,    370
};

uint32_t log2i(const uint32_t x) {
    if( x == 0 ) {
        return 0;
    }
    return (31 - __builtin_clz (x));
}

uint32_t Qlog2( uint8_t x ) {
    return log2table[x];
}

void init_log2_table() {
    for( int i=1; i<256; ++i ) {
        log2table[i] = log2(256./i)*((double)QFACTOR)+0.5;
    }
}

size_t apv2_decoder_get_data_len(const void *src, size_t ssize) {
    APV2 apv2;
    apv2.decompress(NULL, src, ssize);
    return apv2.get_data_len();
}

