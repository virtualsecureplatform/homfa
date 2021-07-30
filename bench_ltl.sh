#!/usr/bin/bash -xeu

spec=(
    'XXXX(!p0 | XXXX(!p1 W X!p2))'
)
input=(
    "test/bench-1200.in"
    "test/bench-2400.in"
    "test/bench-3600.in"
    "test/bench-4800.in"
)

TIME="multitime -n5"
HOMFA=build_rel/bin/homfa

output_file=$(date +'homfa-bench-ltl-%Y%m%d%H%M%S.log')

[ -f _test_sk ] || $HOMFA genkey --out _test_sk
[ -f _test_bk ] || $HOMFA genbkey --key _test_sk --out _test_bk

for s in "${spec[@]}"; do
    for i in "${input[@]}"; do
        echo ">>> $s" >> "$output_file"
        $HOMFA ltl2spec "$s" 3 > _test_spec
        $HOMFA enc --ap 3 --key _test_sk --in "$i" --out _test_in
        #$TIME $HOMFA run-offline-dfa --bkey _test_bk --spec _test_spec --in _test_in --out _test_out 2>&1\
        #    | tee -a "$output_file"
        $TIME $HOMFA run-online-dfa --method qtrlwe2 --bkey _test_bk --spec _test_spec --in _test_in --out _test_out 2>&1\
            | tee -a "$output_file"
        $HOMFA dec --key _test_sk --in _test_out
    done
done
