/*
 * apv2.c for apv2
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

/*
 * The command line is built on nothing but the C++ standard library so the
 * same source builds unchanged on Windows, Linux and macOS.
 */

#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <new>
#include <string>
#include <vector>

#include "APV2.h"
#include "Forward_arrivals_parse.h"
#include "progress_bar.h"
#include "version.h"

namespace fs = std::filesystem;

typedef Forward_arrivals_parse<APV2Cost> APV2_parser;

#define INITIAL_POOL_BYTES (1024 * 1024) /* the parser pool grows dynamically from this size */
#define MAX_INPUT_SIZE (INT32_MAX)   /* the matchfinder works on int32 sizes */

static void print_version()
{
    printf("apv2 %s Copyright (C) 2022 Dirk Helbig\n", apv2_VERSION);
}

static void usage(FILE *out, const char *argv0)
{
    fprintf(out, "Usage: %s [OPTION]... FILE...\n", argv0);
    fprintf(out, "Compress FILEs gzip-style (file -> file.apv2), decompress with -d.\n\n");
    fprintf(out, "  -d, --decompress   decompress (file.apv2 -> file)\n");
    fprintf(out, "  -f, --force        overwrite output file if it exists\n");
    fprintf(out, "  -s, --max-slots N  parser candidate slots per position, 1..%d (default %d)\n",
            INT32_MAX, DEFAULT_MAX_SLOTS);
    fprintf(out, "  -v, --verbose      report progress\n");
    fprintf(out, "  -V, --version      print version and exit\n");
    fprintf(out, "  -h, --help         print this help and exit\n");
    fprintf(out, "\nInvoked as \"unapv2\" (copy or symlink), decompression is the default.\n");
}

static bool parse_max_slots(const char *value, int &maxSlots)
{
    errno = 0;
    char *end = NULL;
    long long parsed = strtoll(value, &end, 10);
    while( *end == ' ' ) ++end;
    if( errno == ERANGE || *end != '\0' || parsed < 1 || parsed > INT32_MAX ) {
        fprintf(stderr, "apv2: invalid slot count '%s' (need an integer from 1 to %d)\n",
                value, INT32_MAX);
        return false;
    }
    maxSlots = (int)parsed;
    return true;
}

static bool long_option_matches(const char *name, const char *option, const char **value)
{
    size_t len = strlen(option);
    if( strncmp(name, option, len) != 0 ) {
        return false;
    }
    if( name[len] == '\0' ) {
        *value = NULL;
        return true;
    }
    if( name[len] == '=' ) {
        *value = name + len + 1;
        return true;
    }
    return false;
}

struct Options {
    bool decompress = false;
    bool force = false;
    bool verbose = false;
    int maxSlots = DEFAULT_MAX_SLOTS;
};

/*
 * Small hand-written parser instead of getopt_long(), which does not exist on
 * Windows. Options may be combined (-dv), long options take --opt=value.
 * Parsing stops at the first non-option; the rest are the input files.
 */
static bool parse_args(int argc, char *args[], Options &opt, int &firstFile)
{
    firstFile = argc;
    for( int i=1; i<argc; ++i ) {
        const char *arg = args[i];
        if( arg[0] != '-' || arg[1] == '\0' ) {
            firstFile = i;
            return true;
        }
        if( arg[1] == '-' ) {
            if( strcmp(arg, "--") == 0 ) {
                firstFile = i+1;
                return true;
            }
            const char *name = arg+2;
            const char *value = NULL;
            if( long_option_matches(name, "decompress", &value) ) {
                opt.decompress = true;
            } else if( long_option_matches(name, "force", &value) ) {
                opt.force = true;
            } else if( long_option_matches(name, "verbose", &value) ) {
                opt.verbose = true;
            } else if( long_option_matches(name, "version", &value) ) {
                print_version();
                exit(EXIT_SUCCESS);
            } else if( long_option_matches(name, "help", &value) ) {
                usage(stdout, args[0]);
                exit(EXIT_SUCCESS);
            } else if( long_option_matches(name, "max-slots", &value) ) {
                if( value == NULL ) {
                    if( i+1 >= argc ) {
                        fprintf(stderr, "apv2: option '--max-slots' requires an argument\n");
                        return false;
                    }
                    value = args[++i];
                }
                if( !parse_max_slots(value, opt.maxSlots) ) {
                    return false;
                }
            } else {
                fprintf(stderr, "apv2: unrecognized option '%s'\n", arg);
                usage(stderr, args[0]);
                return false;
            }
        } else {
            for( const char *c = arg+1; *c != '\0'; ++c ) {
                switch( *c ) {
                case 'd':
                    opt.decompress = true;
                    break;
                case 'f':
                    opt.force = true;
                    break;
                case 'v':
                    opt.verbose = true;
                    break;
                case 'V':
                    print_version();
                    exit(EXIT_SUCCESS);
                case 'h':
                    usage(stdout, args[0]);
                    exit(EXIT_SUCCESS);
                case 's':
                    if( i+1 >= argc ) {
                        fprintf(stderr, "apv2: option requires an argument -- 's'\n");
                        return false;
                    }
                    if( !parse_max_slots(args[++i], opt.maxSlots) ) {
                        return false;
                    }
                    break;
                default:
                    fprintf(stderr, "apv2: invalid option -- '%c'\n", *c);
                    usage(stderr, args[0]);
                    return false;
                }
            }
        }
    }
    return true;
}

static bool read_file(const fs::path &name, std::vector<uint8_t> &buffer)
{
    std::string what = name.string();
    std::error_code ec;
    if( fs::is_directory(name, ec) ) {
        fprintf(stderr, "apv2: %s: is a directory\n", what.c_str());
        return false;
    }
    uintmax_t size = fs::file_size(name, ec);
    if( ec ) {
        fprintf(stderr, "apv2: %s: cannot open input\n", what.c_str());
        return false;
    }
    if( size > MAX_INPUT_SIZE ) {
        fprintf(stderr, "apv2: %s: file too large (max %lu bytes)\n",
                what.c_str(), (unsigned long)MAX_INPUT_SIZE);
        return false;
    }
    std::ifstream in(name, std::ios::binary);
    if( !in ) {
        fprintf(stderr, "apv2: %s: cannot open input\n", what.c_str());
        return false;
    }
    buffer.resize((size_t)size);
    if( size > 0 && !in.read((char *)buffer.data(), (std::streamsize)size) ) {
        fprintf(stderr, "apv2: %s: failed reading input\n", what.c_str());
        return false;
    }
    return true;
}

static bool check_output_free(const fs::path &name, bool force)
{
    std::error_code ec;
    if( !force && fs::exists(name, ec) ) {
        fprintf(stderr, "apv2: %s already exists. Use -f to overwrite.\n", name.string().c_str());
        return false;
    }
    return true;
}

static bool write_file(const fs::path &name, const void *ptr, size_t size)
{
    std::string what = name.string();
    std::ofstream out(name, std::ios::binary | std::ios::trunc);
    if( !out ) {
        fprintf(stderr, "apv2: %s: cannot open output\n", what.c_str());
        return false;
    }
    if( size > 0 && !out.write((const char *)ptr, (std::streamsize)size) ) {
        fprintf(stderr, "apv2: %s: failed writing output\n", what.c_str());
        return false;
    }
    out.close();
    if( !out.good() ) {
        fprintf(stderr, "apv2: %s: failed writing output\n", what.c_str());
        return false;
    }
    return true;
}

static bool compress_file(const fs::path &inFile, const fs::path &outFile,
                          const Options &opt,
                          APV2_parser::progress_callback_t progress)
{
    std::string what = inFile.string();
    std::vector<uint8_t> in;
    if( !read_file(inFile, in) ) {
        return false;
    }
    if( !check_output_free(outFile, opt.force) ) {
        return false;
    }

    std::vector<uint8_t> out;
    double bits = 0;
    try {
        if( in.empty() ) {
            // no tokens at all: flushing the untouched coder decodes back to zero bytes
            // (the coder writes backwards, so hand it the end of the buffer)
            APV2Encode empty(NULL);
            empty.flush();
            out.resize((size_t)empty.get_data_len());
            APV2Encode encode(out.data() + out.size());
            encode.flush();
        } else {
            size_t edges = INITIAL_POOL_BYTES / sizeof(APV2Cost) + 1;
            APV2_parser parser(in.data(), in.size(), edges);
            // the parser price is a Q16 fixed-point bit count
            bits = (double)parser.parse(progress) / QFACTOR;
            out.resize((size_t)apv2_encoder_get_data_len(parser));
            apv2_encode_parse(out.data(), out.size(), parser);
        }
    } catch( const std::bad_alloc & ) {
        progress_bar_done(); // get back to the user's screen before reporting
        fprintf(stderr, "apv2: %s: out of memory\n", what.c_str());
        return false;
    }

    // verify: the compressed data must decode back to the original input, byte for byte
    APV2 decode;
    std::vector<uint8_t> check(in.size());
    bool verified = false;
    try {
        size_t decoded = decode.decompress(check.data(), out.data(), out.size());
        verified = (decoded == in.size())
            && (in.empty() || (memcmp(check.data(), in.data(), in.size()) == 0));
    } catch( const rABS_error & ) {
        verified = false;
    }
    if( !verified ) {
        fprintf(stderr, "apv2: %s: internal error, compressed data failed verification\n", what.c_str());
        return false;
    }

    if( !write_file(outFile, out.data(), out.size()) ) {
        return false;
    }

    double ratio = in.empty() ? 0.0 : (double)out.size() * 100.0 / (double)in.size();
    printf("%s: %.2f%% -- %s %.1f bit %zu B\n",
           what.c_str(), ratio, outFile.string().c_str(), bits, out.size());
    return true;
}

static bool decompress_file(const fs::path &inFile, const fs::path &outFile, const Options &opt)
{
    std::string what = inFile.string();
    std::vector<uint8_t> in;
    if( !read_file(inFile, in) ) {
        return false;
    }
    if( !check_output_free(outFile, opt.force) ) {
        return false;
    }

    // first pass only counts the decoded size, second pass fills the buffer
    APV2 scan;
    size_t outSize = 0;
    try {
        scan.decompress(NULL, in.data(), in.size());
        outSize = scan.get_data_len();
    } catch( const rABS_error & ) {
        fprintf(stderr, "apv2: %s: corrupt input file\n", what.c_str());
        return false;
    }

    std::vector<uint8_t> out(outSize);
    APV2 decode;
    size_t decoded = 0;
    try {
        decoded = decode.decompress(out.data(), in.data(), in.size());
    } catch( const rABS_error & ) {
        fprintf(stderr, "apv2: %s: corrupt input file\n", what.c_str());
        return false;
    }
    if( decoded != outSize ) {
        fprintf(stderr, "apv2: %s: corrupt input file\n", what.c_str());
        return false;
    }

    if( !write_file(outFile, out.data(), out.size()) ) {
        return false;
    }

    double ratio = outSize == 0 ? 0.0 : (double)in.size() * 100.0 / (double)outSize;
    printf("%s: %.2f%% -- %s %zu B\n", what.c_str(), ratio, outFile.string().c_str(), outSize);
    return true;
}

int main( int argc, char *args[] )
{
    Options opt;
    opt.decompress = (fs::path(args[0]).filename().stem() == "unapv2");

    int firstFile = 1;
    if( !parse_args(argc, args, opt, firstFile) ) {
        return EXIT_FAILURE;
    }
    if( firstFile >= argc ) {
        fprintf(stderr, "apv2: too few arguments\n");
        usage(stderr, args[0]);
        return EXIT_FAILURE;
    }

    APV2Cost::maxSlots = opt.maxSlots;

    // the progress bar redraws its line, so it only makes sense on a terminal
    APV2_parser::progress_callback_t progress = NULL;
    if( opt.verbose && console_columns() > 0 ) {
        progress = print_progress;
        progress_bar_init();
    }

    int status = EXIT_SUCCESS;
    for( int j=firstFile; j<argc; ++j ) {
        fs::path inFile(args[j]);
        fs::path outFile(inFile);

        if( !opt.decompress && inFile.extension() == ".apv2" ) {
            fprintf(stderr, "apv2: %s already has .apv2 suffix -- unchanged\n", inFile.string().c_str());
            continue; // like gzip: the file is already in the wanted form, not an error
        }
        if( opt.decompress && inFile.extension() != ".apv2" ) {
            fprintf(stderr, "apv2: %s: unknown suffix -- ignored\n", inFile.string().c_str());
            status = status < 2 ? 2 : status; // gzip exits 2 for a wrong suffix
            continue;
        }
        if( opt.decompress ) {
            outFile.replace_extension();
        } else {
            outFile += ".apv2";
        }

        bool ok = opt.decompress ? decompress_file(inFile, outFile, opt)
                                 : compress_file(inFile, outFile, opt, progress);
        if( !ok && status == EXIT_SUCCESS ) {
            status = EXIT_FAILURE;
        }
    }
    return status;
}
