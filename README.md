# HomFA

HomFA enables you to run a DFA __obliviously__, i.e., without revealing any information about its input and structure. Please see [our paper](#Publication) for the details.

## On the Artifact Submitted to CAV'22

If you would like to obtain the artifact we submitted to CAV'22 (34th International Conference on Computer Aided Verification), plaese visit [here](https://doi.org/10.5281/zenodo.6558657) (faster mirrors: [Backblaze B2 us-west](https://anqou-share.s3.us-west-000.backblazeb2.com/homfa_cav22_1.zip) and [Scaleway pl-waw](https://anqou-share.s3.pl-waw.scw.cloud/homfa_cav22_1.zip)). This GitHub repository is used for our development and is not fully documented, while the artifact has a detailed README and a Docker image. You may also find [this repo](https://github.com/virtualsecureplatform/homfa-cav22) interesting, which is used to generate our artifact.

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

## To Enable Profiling by Pprof

Build with a CMake option `-DHOMFA_ENABLE_PROFILE=On`.
Then use environment variables `HEAPPROFILE=filename` and `CPUPROFILE=filename`.

## Publication

- Ryotaro Banno, Kotaro Matsuoka, Naoki Matsumoto, Song Bian, Masaki Waga, & Kohei Suenaga. (2022). Oblivious Online Monitoring for Safety LTL Specification via Fully Homomorphic Encryption.
  - To appear in [CAV'22](http://i-cav.org/2022/).
  - Extended version on [arXiv](https://arxiv.org/abs/2206.03582) and [lab](https://www.fos.kuis.kyoto-u.ac.jp/~banno/cav22.pdf).
  - > In many Internet of Things (IoT) applications, data sensed by an IoT device are continuously sent to the server and monitored against a specification. Since the data often contain sensitive information, and the monitored specification is usually proprietary, both must be kept private from the other end. We propose a protocol to conduct oblivious online monitoring -- online monitoring conducted without revealing the private information of each party to the other -- against a safety LTL specification. In our protocol, we first convert a safety LTL formula into a DFA and conduct online monitoring with the DFA. Based on fully homomorphic encryption (FHE), we propose two online algorithms (Reverse and Block) to run a DFA obliviously. We prove the correctness and security of our entire protocol. We also show the scalability of our algorithms theoretically and empirically. Our case study shows that our algorithms are fast enough to monitor blood glucose levels online, demonstrating our protocol's practical relevance.
