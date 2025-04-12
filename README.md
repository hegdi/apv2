This is an implementation of the APV2 compression format as described by @yupferris in his
[blog](https://yupferris.github.io/blog/2020/08/31/c64-4k-intro-packer-deep-dive)
post.

The source code is intended to be a easy to use framework to test compression formats.
It strives for the highest compression the format can achieve, without memory
or computation constrains. Thus it is slow and needs allot of memory.

The APV2 sample compression generates smaller files than the usual 
memory/computation constraint compressors like
[Exomizer](https://bitbucket.org/magli143/exomizer/wiki/Home)
or intro compressors like
[Shrinkler](https://github.com/askeksa/Shrinkler) or
[admiral-p4kbar](https://github.com/logicomacorp/makeshift/tree/new-packer/admiral-p4kbar)
( which is the reference implementation of APV2 )
although ignoring things like integrity or any sanity checks.
Only lacks against small size context mixing compressors like
[Crinkler](https://github.com/runestubbe/Crinkler).

Not intended to be used in real world scenarios or to replace any of the above
mentioned compression tools. You have been warned.

Thanks to @yupferris for the nice format.
Thanks to @emmanuel-marty for his work, as this is basically a simplified version of his
parser and of course @IlyaGrebnov for all about suffix array construction.

# build
```sh
> cmake -B build
> make -C build
```

# usage
```sh
> ./apv2 <filename>
```
