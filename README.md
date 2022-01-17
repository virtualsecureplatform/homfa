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

