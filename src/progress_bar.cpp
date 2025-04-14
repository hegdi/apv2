/*
 * progress_bar.c for apv2
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
#include <algorithm>
#include <cstdio>
#include <sys/ioctl.h>

#include "humanize_number.h"
#include "progress_bar.h"

static unsigned get_columns() {
    struct winsize sz;
    ioctl( 0, TIOCGWINSZ, &sz );
    return sz.ws_col;
}

static const char style[] = "[= ]";
#define SPINNER_SIZE 4
static const char spinner[SPINNER_SIZE+1] = "|/-\\";
enum {
    BAR_START    = 0,
    BAR_PROGRESS = 1,
    BAR_EMPTY    = 2,
    BAR_END      = 3
};

#define PERCENTAGE(V, T) (V*100/T)
#define DESCRIPTION_LENGTH (28)

void print_progress(size_t count, size_t max, size_t poolLevel, size_t poolSize) {
    char remainingBytes[8];
    const char *Unit = "B";
    humanize_number( remainingBytes, sizeof(remainingBytes), poolLevel, Unit, HN_AUTOSCALE, 0);
    char poolSizeBytes[8];
    humanize_number( poolSizeBytes, sizeof(poolSizeBytes), poolSize, Unit, HN_AUTOSCALE, 0);

    unsigned current_columns = get_columns();
    unsigned bar_width = std::max((int)current_columns-DESCRIPTION_LENGTH, 0);
    bar_width = std::min(bar_width, 100u);
    unsigned bar_percent = (count*bar_width)/max;

    static char bar[100+1] = "";
    unsigned i;
    for (i=0; i < bar_width; ++i) {
        bar[i] = i < bar_percent ? style[BAR_PROGRESS] : style[BAR_EMPTY];
    }
    bar[i] = 0;

    static int spin = 0;
    spin++;
    spin %= SPINNER_SIZE;

    printf("\e[2K\r%c%c%c%c%s%c %3d%% %8s %8s", style[BAR_START], spinner[spin], style[BAR_END], style[BAR_START], bar, style[BAR_END], (unsigned char)PERCENTAGE(count, max), remainingBytes, poolSizeBytes);
    if( count == max ) {
        printf("\e[2K\r");
    }
    fflush(stdout);
}

