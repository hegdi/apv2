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

#include <cassert>
#include <cinttypes>
#include <filesystem>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "APV2.h"
#include "Forward_arrivals_parse.h"
#include "humanize_number.h"
#include "progress_bar.h"
#include "version.h"

static void dump_to_file( const char *file, const void* ptr, size_t size )
{
    FILE *fd = fopen( file, "wb" );
    if(fd == NULL ) {
        perror("can't open file");
        return;
    }
    fwrite( ptr, size, 1, fd );
    fclose( fd );
}

static uint8_t *mmap_file( const char *file, size_t *len )
{
    struct stat sb;
    int fd = open(file, O_RDONLY);
    if (fd < 0) {
        perror("can't open file");
        return NULL;
    }

    if (fstat(fd, &sb) < 0) { /* To obtain file size */
        perror("can't get file size");
        close(fd);
        return NULL;
    }
    size_t fileSize = sb.st_size;
    *len = fileSize;
    assert( fileSize <= INT32_MAX );

    uint8_t *addr = (uint8_t*)mmap(NULL, fileSize, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    if( addr == MAP_FAILED ) {
        perror("can't mmap file");
        return NULL;
    }
    return addr;
}

static void unmap_file( void *addr, size_t len )
{
    munmap(addr, len);
}

static const char short_options[] = "dp:vVh";
static const struct option long_options[] = {
    {"decompress",       no_argument, NULL, 'd'},
    {"pool_size",  required_argument, NULL, 'p'},
    {"verbose",          no_argument, NULL, 'v'},
    {"version",          no_argument, NULL, 'V'},
    {"help",             no_argument, NULL, 'h'},
    {NULL,                         0, NULL,  0 },
};

static const char *help_options[] = {
    "decompress",
    "initial edge pool size in MB",
    "verbose mode",
    "display version number",
    "give this help",
};

static void print_version()
{
    printf("%s Copyright (C) 2022 Dirk Helbig\n", apv2_VERSION);
}

static void usage(std::filesystem::path file)
{
    const char *filename = file.filename().c_str();
    printf("%s ", filename);
    print_version();
    printf("Usage: %s [OPTION]... [FILE]...\n", filename);
    printf("Compress or decompress FILEs (by default, compress FILES in-place).\n");
    for( int i=0; long_options[i].name != 0; i++) {
        printf("--%s|-%c\t\t%s\n", long_options[i].name, long_options[i].val, help_options[i] );
    }
}

int main( int argc, char *args[] )
{
    int opt;
    bool decompress = (!strcmp(args[0],"unapv2"))?true:false;
    bool verbose = false;
    ssize_t pool_size = 1024*1024*1024;
    while ((opt = getopt_long(argc, args, short_options, long_options, NULL)) != -1) {
        switch (opt) {
        case 'd':
            decompress = true;
            break;
        case 'p':
            pool_size = atoi(optarg)*1024*1024;
            break;
        case 'v':
            verbose = true;
            break;
        case 'V':
            print_version();
            return EXIT_SUCCESS;
        case 'h':
            /* fall through */
        default:
            usage(std::filesystem::path(args[0]));
            return EXIT_SUCCESS;
        }
    }

    if (argc == optind) {
        fprintf(stderr, "%s: too few arguments.\n", args[0]);
        usage(args[0]);
        return EXIT_FAILURE;
    }
    if( pool_size <= 0 ) {
        fprintf(stderr, "%s: pool size needs to be greater than zero.\n", args[0]);
        return EXIT_FAILURE;
    }

    char poolSizeBytes[8];
    const char *Unit = "B";
    humanize_number( poolSizeBytes, sizeof(poolSizeBytes), pool_size, Unit, HN_AUTOSCALE, 0);
    if( verbose ) {
        printf("initial memory pool size: %s\n", poolSizeBytes);
    }

//    init_log2_table();
    for( int j=optind; j<argc; ++j ) {
        std::filesystem::path outFile = args[j];
        std::filesystem::path inFile = args[j];

        if( !decompress && (inFile.extension() == ".apv2") ) {
            fprintf(stderr, "%s: %s already has .apv2 suffix -- unchanged\n", args[0], inFile.c_str());
            continue;
        }
        if( decompress && (inFile.extension() != ".apv2") ) {
            fprintf(stderr, "%s: %s: unknown suffix -- ignored\n", args[0], inFile.c_str());
            continue;
        }
        if( decompress ) {
            outFile.replace_extension();
        } else {
            outFile += ".apv2";
        }

        size_t inFileSize;
        uint8_t *addr = mmap_file(inFile.c_str(), &inFileSize);
        if( addr == NULL ) {
            return EXIT_FAILURE;
        }
        uint8_t *outBuf = NULL;
        size_t outFileSize = 0;
        if( !decompress ) { // Compress
            typedef Forward_arrivals_parse<APV2Cost> APV2_parser;
            APV2_parser parser(addr, inFileSize, pool_size/sizeof(APV2Cost)+1);
            APV2_parser::progress_callback_t progress = NULL;
            if( verbose ) {
                progress = print_progress;
            }
            size_t compressedBits = parser.parse(progress);
            outFileSize = apv2_encoder_get_data_len(parser);
            outBuf = new uint8_t[outFileSize];
            apv2_encode_parse( outBuf, outFileSize, parser );
            printf("%s:\t%3.2f\%% -- %s %f bit %zd B ", inFile.c_str(), ((float)outFileSize*100)/inFileSize, outFile.c_str(), QTOF(compressedBits), outFileSize);

            // verify
            uint8_t *verifyBuf = new uint8_t[inFileSize];
            APV2 decode;
            decode.decompress( verifyBuf, outBuf, outFileSize );
            // force equality
            for( size_t i=0; i<inFileSize; ++i ) {
                if( verifyBuf[i]!=addr[i]) {
                    printf("failed!\n");
                    return EXIT_FAILURE;
                }
            }
            printf("\n");
            delete [] verifyBuf;
        } else { // Decompress
            outFileSize = apv2_decoder_get_data_len( addr, inFileSize );
            outBuf = new uint8_t[outFileSize];
            APV2 decode;
            decode.decompress( outBuf, addr, inFileSize );
            printf("%s:\t%3.2f\%% -- %s %zu B\n", inFile.c_str(), ((float)inFileSize*100)/outFileSize, outFile.c_str(), outFileSize);
        }
        dump_to_file( outFile.c_str(), outBuf, outFileSize);
        unmap_file(addr, inFileSize);
        delete [] outBuf;
    }
    return EXIT_SUCCESS;
}

