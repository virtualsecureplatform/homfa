# HomFA

Homomorphic Final Answer

## Build and run

```sh
$ mkdir build
$ cd build
$ cmake ..
$ make
$ bin/homfa genkey --out sk
$ bin/homfa genbkey --key sk --out bk
$ bin/homfa enc --key sk --in ../test/01-01.in --out enc_in
$ bin/homfa run-offline-dfa --bkey bk --spec ../test/01.spec --in enc_in --out enc_out
$ bin/homfa dec --key sk --in enc_out
```

```sh

./homfa ltl2spec 'G((!p0 & !p1 & !p2 &  p3 &  p4) | ( p0 &  p1 &  p2 & !p3 &  p4) | (!p0 &  p1 &  p2 & !p3 &  p4) | ( p0 & !p1 &  p2 & !p3 &  p4) | (!p0 & !p1 &  p2 & !p3 &  p4) | ( p0 &  p1 & !p2 & !p3 &  p4) | (!p0 &  p1 & !p2 & !p3 &  p4) | ( p0 & !p1 & !p2 & !p3 &  p4) | (!p0 & !p1 & !p2 & !p3 &  p4) | ( p0 &  p1 &  p2 &  p3 & !p4) | (!p0 &  p1 &  p2 &  p3 & !p4) | ( p0 & !p1 &  p2 &  p3 & !p4))' 5 | \
./homfa spec2spec --minimized | \
./homfa spec2dot | \
dot -Tpng > ../../graph.png
```

## Build 色々

Ubuntu 20.04 LTS なら下のコマンドでだいたい間に合う。全て`build`ディレクトリ内での実行を想定。

```sh
# homfaとtest0をリリースビルド
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10 ..
# あるいは
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang-10 -DCMAKE_CXX_COMPILER=clang++-10 ..

# homfaとtest0とtest_plain_randomとtest_crypto_randomをリリースビルド
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10 -DHOMFA_BUILD_TEST_PLAIN_RANDOM=On -DHOMFA_BUILD_TEST_CRYPTO_RANDOM=On ..
# あるいは
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang-10 -DCMAKE_CXX_COMPILER=clang++-10 -DHOMFA_BUILD_TEST_PLAIN_RANDOM=On -DHOMFA_BUILD_TEST_CRYPTO_RANDOM=On ..
```
