#!/usr/bin/bash -xeu

spec=(
    "test/bench-1024.spec"
    "test/bench-256.spec"
    "test/bench-64.spec"
    "test/bench-128.spec"
    "test/bench-512.spec"
)

input=(
    "test/bench-1024.in"
    "test/bench-16384.in"
    "test/bench-2048.in"
    "test/bench-4096.in"
    "test/bench-8192.in"
)

TIME="multitime -n5"
HOMFA=build/bin/homfa

output_file=$(date +'homfa-bench-%Y%m%d%H%M%S.log')

for s in "${spec[@]}"; do
    for i in "${input[@]}"; do
        $TIME $HOMFA "$s" "$i" 2>&1 | tee -a "$output_file"
    done
done
