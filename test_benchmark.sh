#!/usr/bin/bash -eux

BUILD_BIN=${BUILD_BIN:-"build/bin"}
BENCHMARK=$BUILD_BIN/benchmark
OUTPUT_FREQ=30
QTRLWE2_QUEUE_SIZE=15
QTRLWE2_MAX_SECOND_LUT_DEPTH=8

failwith(){
    echo -ne "\e[1;31m[ERROR]\e[0m "
    echo "$1"
    exit 1
}

run_benchmark(){
    local mode=$1
    local ap_size=$2
    local spec_filepath=$3
    local input_filepath=$4
    local output_freq=$5

    case $mode in
        "offline" )
            $BENCHMARK offline \
                --spec $spec_filepath \
                --in $input_filepath \
                --bootstrapping-freq 30000 \
                --ap $ap_size \
                > _test_log
            ;;
        "reversed" )
            $BENCHMARK reversed \
                --spec $spec_filepath \
                --in $input_filepath \
                --bootstrapping-freq 30000 \
                --out-freq $output_freq \
                --ap $ap_size \
                --spec-reversed \
                > _test_log
            ;;
        "qtrlwe2" | "flut" )
            $BENCHMARK qtrlwe2 \
                --spec $spec_filepath \
                --in $input_filepath \
                --bootstrapping-freq 1 \
                --out-freq $output_freq \
                --ap $ap_size \
                --queue-size $QTRLWE2_QUEUE_SIZE \
                --max-second-lut-depth $QTRLWE2_MAX_SECOND_LUT_DEPTH \
                > _test_log
            ;;
        "bbs" )
            $BENCHMARK bbs \
                --spec $spec_filepath \
                --in $input_filepath \
                --out-freq $output_freq \
                --ap $ap_size \
                --queue-size $output_freq \
                > _test_log
            ;;
        "plain" )
            $BENCHMARK plain \
                --spec $spec_filepath \
                --in $input_filepath \
                --out-freq $output_freq \
                --ap $ap_size \
                > _test_log
            ;;
        * )
            failwith "Invalid run $1"
            ;;
    esac
}

run_test(){
    local ap_size=$1
    local spec_filepath=$2
    local spec_rev_filepath=$3
    local input_filepath=$4

    run_benchmark plain $ap_size $spec_filepath $input_filepath 1
    cat _test_log | grep "result" | tail -1 > _test_log_plain_last
    run_benchmark plain $ap_size $spec_filepath $input_filepath $OUTPUT_FREQ
    cat _test_log | grep "result" > _test_log_plain

    run_benchmark offline $ap_size $spec_filepath $input_filepath 0
    diff _test_log_plain_last <(cat _test_log | grep "result")
    run_benchmark reversed $ap_size $spec_rev_filepath $input_filepath $OUTPUT_FREQ
    diff _test_log_plain <(cat _test_log | grep "result")
    run_benchmark flut $ap_size $spec_filepath $input_filepath $OUTPUT_FREQ
    diff _test_log_plain <(cat _test_log | grep "result")
    run_benchmark bbs $ap_size $spec_filepath $input_filepath $OUTPUT_FREQ
    diff _test_log_plain <(cat _test_log | grep "result")
}

run_test 2 test/01.spec test/01_rev.spec test/01-01.in
run_test 2 test/01.spec test/01_rev.spec test/01-02.in
run_test 2 test/01.spec test/01_rev.spec test/01-03.in
run_test 2 test/01.spec test/01_rev.spec test/01-04.in
run_test 2 test/01.spec test/01_rev.spec test/01-05.in
run_test 2 test/01.spec test/01_rev.spec test/01-06.in

