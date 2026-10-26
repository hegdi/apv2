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

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "humanize_number.h"
#include "progress_bar.h"

/* the one platform selection: the terminal size and its escape-sequence mode
 * have no standard API */
#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>

unsigned console_columns(void)
{
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
    if( out == NULL || out == INVALID_HANDLE_VALUE
        || !GetConsoleScreenBufferInfo(out, &csbi) ) {
        return 0;
    }
    return (unsigned)(csbi.srWindow.Right - csbi.srWindow.Left + 1);
}

int console_ansi(void)
{
    static int supported = -1;
    if( supported < 0 ) {
        HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD mode;
        supported = (GetConsoleMode(out, &mode) != 0
                     && SetConsoleMode(out, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) != 0);
    }
    return supported;
}

#else
#  include <sys/ioctl.h>
#  include <unistd.h>

unsigned console_columns(void)
{
    struct winsize ws;
    if( ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) != 0 ) {
        return 0;
    }
    return ws.ws_col;
}

int console_ansi(void)
{
    const char *term = getenv("TERM");
    return term == NULL || strcmp(term, "dumb") != 0;
}

#endif

/*
 * The bar lives inline on the user's own screen, always the last thing
 * written. Every frame is a complete repaint of the bar's block: climb to
 * its top (a resize may have re-wrapped the previous frame over several
 * rows - the cursor always ends up at the text's new end, so this is
 * predictable), erase everything from there down and draw the new frame at
 * the current terminal width. Whatever snapping or reflow happened in
 * between is wiped before it can pile up, and the finished bar remains in
 * the scrollback like gzip-style output should.
 */
static size_t drawnLen = 0;  /* chars of the frame currently on screen */
static int barShown = 0;

void progress_bar_done(void)
{
    if( barShown ) {
        barShown = 0;
        drawnLen = 0;
        fflush(stdout);
        fputc('\n', stdout);
        fflush(stdout);
    }
}

static void sigint_handler(int sig)
{
    signal(sig, SIG_DFL);
    progress_bar_done();
    raise(sig);
}

void progress_bar_init(void)
{
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
}

void print_progress(size_t count, size_t max, size_t poolLevel, size_t poolSize)
{
    // redraw at most ~25 times per second, the last frame always goes out
    static std::chrono::steady_clock::time_point lastDraw;
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if( count < max && now - lastDraw < std::chrono::milliseconds(40) ) {
        return;
    }
    lastDraw = now;

    char level[8];
    char total[8];
    // humanize_number() scales the value so it fits the buffer width
    humanize_number(level, sizeof(level), (int64_t)poolLevel, "B", HN_AUTOSCALE, 0);
    humanize_number(total, sizeof(total), (int64_t)poolSize, "B", HN_AUTOSCALE, 0);

    // everything around the bar itself: "| [..] 100% 1023 MB free of 1024 MB"
    static const size_t description = 35;
    static const size_t minBar = 8;
    static const size_t maxBar = 100;
    static char bar[maxBar + 1];
    static const char spinner[] = "|/-\\";
    static size_t spin = 0;

    unsigned columns = console_columns();
    if( columns == 0 ) {
        columns = 80; // terminal of unknown size, e.g. TERM=dumb
    }
    // keep the terminal's last column free: a line that fills the terminal
    // exactly leaves the cursor in a wrap state the next query cannot resolve
    size_t lineRoom = columns > 1 ? columns - 1 : 1;
    size_t barWidth = (lineRoom > description + minBar) ? lineRoom - description : minBar;
    if( barWidth > maxBar ) {
        barWidth = maxBar;
    }

    size_t filled = (max > 0) ? (count * barWidth) / max : 0;
    size_t i = 0;
    for( ; i < filled && i < barWidth; ++i ) {
        bar[i] = '=';
    }
    for( ; i < barWidth; ++i ) {
        bar[i] = ' ';
    }
    bar[barWidth] = '\0';

    unsigned percent = (max > 0) ? (unsigned)((count * 100) / max) : 0;
    char line[160];
    snprintf(line, sizeof(line), "%c [%s] %3u%% %6s free of %s",
             spinner[spin % 4], bar, percent, level, total);
    ++spin;

    if( console_ansi() ) {
        // climb to the top of the block the previous frame now occupies -
        // a resize may have re-wrapped it - erase from there down and
        // repaint at the current width; nothing the terminal did survives
        size_t rows = (drawnLen + columns - 1) / columns;
        if( rows > 1 ) {
            printf("\033[%zuA", rows - 1);
        }
        printf("\r\033[0J%s", line);
        drawnLen = strlen(line);
    } else {
        // without escapes only the current row can be overwritten: pad the
        // remainder of a previously longer line away with spaces
        static size_t lastLength = 0;
        size_t length = strlen(line);
        printf("\r%s", line);
        for( size_t pad = length; pad < lastLength; ++pad ) {
            putchar(' ');
        }
        lastLength = length > lastLength ? length : lastLength;
    }
    barShown = 1;
    fflush(stdout);
    if( count >= max ) {
        // the final frame joins the output history, like a gzip-style line
        barShown = 0;
        drawnLen = 0;
        printf("\n");
        fflush(stdout);
    }
}
