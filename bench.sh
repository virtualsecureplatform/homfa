#!/usr/bin/bash -xeu

spec=(
    "test/bench-64.spec"
    "test/bench-128.spec"
    "test/bench-256.spec"
    "test/bench-512.spec"
    "test/bench-1024.spec"
)

input=(
    "test/bench-1024.in"
    "test/bench-2048.in"
    "test/bench-4096.in"
    "test/bench-8192.in"
    "test/bench-16384.in"
)

TIME="multitime -n5"
HOMFA=build_rel/bin/homfa

output_file=$(date +'homfa-bench-%Y%m%d%H%M%S.log')

[ -f _test_sk ] || $HOMFA genkey --out _test_sk
[ -f _test_bk ] || $HOMFA genbkey --key _test_sk --out _test_bk

for s in "${spec[@]}"; do
    for i in "${input[@]}"; do
        $HOMFA enc --key _test_sk --ap 1 --in "$i" --out _test_in
        $TIME $HOMFA run-offline-dfa --bkey _test_bk --spec "$s" --in _test_in --out _test_out 2>&1\
            | tee -a "$output_file"
        $HOMFA dec --key _test_sk --in _test_out | grep "Result (bool): true"
        [ $? -eq 0 ] || exit 1
    done
done
