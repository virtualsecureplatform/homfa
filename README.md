# HomFA

HomFA enables you to run a DFA __obliviously__, i.e., without revealing any information about its input and structure. Please see [our paper](#Publication) for the details.

## Notice on CAV'22

If you would like to obtain the artifact we submitted to CAV'22 (34th International Conference on Computer Aided Verification), plaese visit [here](https://doi.org/10.5281/zenodo.6558657) (Faster mirrors: [Backblaze B2 us-west](https://anqou-share.s3.us-west-000.backblazeb2.com/homfa_cav22_1.zip) and [Scaleway pl-waw](https://anqou-share.s3.pl-waw.scw.cloud/homfa_cav22_1.zip)). This GitHub repository is used for our development and is not fully documented, while the artifact has a detailed README and a Docker image. You may also find [this repo](https://github.com/virtualsecureplatform/homfa-cav22) interesting, which is used to generate our artifact.

## Build and Run

We use Ubuntu 20.04.4 LTS for our development.
Please install Clang 10 and [Spot 2.9.7](http://www.lrde.epita.fr/dload/spot/spot-2.9.7.tar.gz) in advance.

```sh
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang-10 -DCMAKE_CXX_COMPILER=clang++-10 ..
$ make
$ bin/homfa genkey --out sk
$ bin/homfa genbkey --key sk --out bk
$ bin/homfa enc --key sk --in ../test/01-01.in --out enc_in --ap 2
$ bin/homfa run offline --bkey bk --spec ../test/01.spec --in enc_in --out enc_out --bootstrapping-freq 100000
$ bin/homfa dec --key sk --in enc_out
```

```sh
./homfa ltl2spec 'G((!p0 & !p1 & !p2 &  p3 &  p4) | ( p0 &  p1 &  p2 & !p3 &  p4) | (!p0 &  p1 &  p2 & !p3 &  p4) | ( p0 & !p1 &  p2 & !p3 &  p4) | (!p0 & !p1 &  p2 & !p3 &  p4) | ( p0 &  p1 & !p2 & !p3 &  p4) | (!p0 &  p1 & !p2 & !p3 &  p4) | ( p0 & !p1 & !p2 & !p3 &  p4) | (!p0 & !p1 & !p2 & !p3 &  p4) | ( p0 &  p1 &  p2 &  p3 & !p4) | (!p0 &  p1 &  p2 &  p3 & !p4) | ( p0 & !p1 &  p2 &  p3 & !p4))' 5 | \
./homfa spec2spec --minimized | \
./homfa spec2dot | \
dot -Tpng > ../../graph.png
```

## Enable Profiling by Pprof

Build with a CMake option `-DHOMFA_ENABLE_PROFILE=On`.
Then use environment variables `HEAPPROFILE=filename` and `CPUPROFILE=filename`.

## Publication

- Ryotaro Banno, Kotaro Matsuoka, Naoki Matsumoto, Song Bian, Masaki Waga, & Kohei Suenaga. (2022). Oblivious Online Monitoring for Safety LTL Specification via Fully Homomorphic Encryption.
  - To appear in [CAV'22](http://i-cav.org/2022/).
  - Extended version on [arXiv](https://arxiv.org/abs/2206.03582) and [lab](https://www.fos.kuis.kyoto-u.ac.jp/~banno/cav22.pdf).
