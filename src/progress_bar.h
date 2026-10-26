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

#ifndef PROGRESS_BAR_H_
#define PROGRESS_BAR_H_

#include <stddef.h>

/*
 * Columns of the terminal attached to stdout, 0 when stdout is not a
 * terminal. Querying it on every redraw is also how a resized terminal is
 * picked up. The OS call differs per platform, hence the one #ifdef in the
 * codebase (see progress_bar.cpp); failing the call doubles as the tty check.
 */
unsigned console_columns(void);

/*
 * 1 when ANSI escapes may be printed to stdout, 0 for plain-text only.
 * Windows consoles only became real terminals with Windows 10 and need a
 * one-time mode flag; the same platform block as above covers it.
 */
int console_ansi(void);

/*
 * Ends the progress display: the last bar frame is kept as an ordinary
 * output line and the cursor moves below it. Any code that may abort
 * parsing and print a message afterwards should call it first so the
 * message starts on a free line. Idempotent.
 */
void progress_bar_done(void);

/*
 * Call once from main when the progress bar will be used: while a bar is
 * on screen, Ctrl+C (or SIGTERM) ends it cleanly - the terminal is left
 * with the bar as a finished line - before the process dies.
 */
void progress_bar_init(void);

void print_progress(size_t count, size_t max, size_t poolLevel, size_t poolSize);

#endif /* PROGRESS_BAR_H_ */
