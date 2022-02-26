#!/usr/bin/bash -eux

BUILD_BIN=${BUILD_BIN:-"build/bin"}
TEST0=$BUILD_BIN/test0
TEST_PLAIN_RANDOM=$BUILD_BIN/test_plain_random
HOMFA=$BUILD_BIN/homfa
OFFLINE_BOOTSTRAPPING_FREQ=8000
REVERSE_BOOTSTRAPPING_FREQ=8000
FLUT_MAX_SECOND_LUT_DEPTH=8
FLUT_QUEUE_SIZE=15
OUTPUT_FREQ=15

failwith(){
    echo -ne "\e[1;31m[ERROR]\e[0m "
    echo "$1"
    exit 1
}

nostderr(){
    "$@" 2>> _test_stderr
    local code=$?
    [ $code -eq 0 ] || failwith "Exit code: $code"
}

enc_run_dec(){
    case "$1" in
        "offline-dfa" )
            nostderr $HOMFA enc --ap "$2" --key _test_sk --in "$4" --out _test_in
            nostderr $HOMFA run offline --bkey _test_bk --spec "$3" --in _test_in --out _test_out --bootstrapping-freq $OFFLINE_BOOTSTRAPPING_FREQ
            nostderr $HOMFA dec --key _test_sk --in _test_out
            ;;
        "online-dfa-reversed" )
            nostderr $HOMFA enc --ap "$2" --key _test_sk --in "$4" --out _test_in
            nostderr $HOMFA run reversed --bkey _test_bk --spec "$3" --in _test_in --out _test_out --out-freq $OUTPUT_FREQ --bootstrapping-freq $REVERSE_BOOTSTRAPPING_FREQ
            nostderr $HOMFA dec --key _test_sk --in _test_out
            ;;
        "online-dfa-reversed-with-rev-spec" )
            nostderr $HOMFA enc --ap "$2" --key _test_sk --in "$4" --out _test_in
            nostderr $HOMFA run reversed --bkey _test_bk --spec "$3" --in _test_in --out _test_out --spec-reversed --out-freq $OUTPUT_FREQ --bootstrapping-freq $REVERSE_BOOTSTRAPPING_FREQ
            nostderr $HOMFA dec --key _test_sk --in _test_out
            ;;
        "online-dfa-qtrlwe2" )
            nostderr $HOMFA enc --ap "$2" --key _test_sk --in "$4" --out _test_in
            nostderr $HOMFA run flut --bkey _test_bk --spec "$3" --in _test_in --out _test_out --out-freq $OUTPUT_FREQ --max-second-lut-depth $FLUT_MAX_SECOND_LUT_DEPTH --queue-size $FLUT_QUEUE_SIZE --bootstrapping-freq 1
            nostderr $HOMFA dec --key _test_sk --in _test_out
            ;;
        "dfa-plain" )
            nostderr $HOMFA run plain --ap "$2" --spec "$3" --in "$4"
            ;;
        "online-dfa-blockbackstream" )
            nostderr $HOMFA enc --ap "$2" --key _test_sk --in "$4" --out _test_in
            nostderr $HOMFA run block --bkey _test_bk --spec "$3" --in _test_in --out _test_out --out-freq $OUTPUT_FREQ --queue-size $OUTPUT_FREQ
            nostderr $HOMFA dec --key _test_sk --in _test_out
            ;;
        * )
            failwith "Invalid run $1"
            ;;
    esac
}

check_true(){
    res=$(enc_run_dec "$1" "$2" "$3" "$4")
    [ $res = "1" ] || failwith "Expected true, got false >>> enc_run_dec \"$1\" \"$2\" \"$3\" \"$4\"\\$res"
}

check_false(){
    res=$(enc_run_dec "$1" "$2" "$3" "$4")
    [ $res = "0" ] || failwith "Expected false, got true >>> enc_run_dec \"$1\" \"$2\" \"$3\" \"$4\"\\$res"
}

### Now start testing
$TEST0
#cat test/safety_ltl_5ap.txt | $TEST_PLAIN_RANDOM 5 _test_random.log

#### spec <-> AT&T
#### FIXME: more tests
res=$(cat test/01.spec | $HOMFA spec2att | $HOMFA att2spec | wc -l)
if [ $res -ne 9 ]; then
    failwith "spec2att or att2spec failed"
fi

### Prepare secret key and bootstrapping key
[ -f _test_sk ] || nostderr $HOMFA genkey --out _test_sk
[ -f _test_bk ] || nostderr $HOMFA genbkey --key _test_sk --out _test_bk

#### Plain DFA
check_true  dfa-plain 2 test/01.spec test/01-01.in # [1, 1] * 8 * 100
check_false dfa-plain 2 test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  dfa-plain 2 test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20
check_false dfa-plain 2 test/01.spec test/01-04.in # [1, 0] + ([0, 0, 1, 0] * 8 * 20) + [0, 1] + ([0, 0, 0, 1] * 8 * 20)
check_true  dfa-plain 2 test/01.spec test/01-05.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 100
check_false dfa-plain 2 test/01.spec test/01-06.in # [1, 0] + ([0, 0, 1, 0] * 8 * 100) + [0, 1] + ([0, 0, 0, 1] * 8 * 150)
check_true  dfa-plain 2 test/01.spec test/01-07.in # [1, 1] * 4
check_false dfa-plain 2 test/01.spec test/01-08.in # [1, 0] * 4
check_true  dfa-plain 9 test/10.spec test/10-01.in # "111111111" * 100
check_false dfa-plain 9 test/10.spec test/10-02.in # "111111110" * 100
check_true  dfa-plain 9 test/10.spec test/10-03.in # "111111110" * 90

#### Offline DFA
check_true  offline-dfa 2 test/01.spec test/01-01.in # [1, 1] * 8 * 100
check_false offline-dfa 2 test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  offline-dfa 2 test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20
check_false offline-dfa 2 test/01.spec test/01-04.in # [1, 0] + ([0, 0, 1, 0] * 8 * 20) + [0, 1] + ([0, 0, 0, 1] * 8 * 20)
check_true  offline-dfa 2 test/01.spec test/01-05.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 100
check_false offline-dfa 2 test/01.spec test/01-06.in # [1, 0] + ([0, 0, 1, 0] * 8 * 100) + [0, 1] + ([0, 0, 0, 1] * 8 * 150)
check_true  offline-dfa 9 test/10.spec test/10-01.in # "111111111" * 100
check_false offline-dfa 9 test/10.spec test/10-02.in # "111111110" * 100
check_true  offline-dfa 9 test/10.spec test/10-03.in # "111111110" * 90

#### Online DFA (reversed)
check_true  online-dfa-reversed 2 test/01.spec test/01-07.in # [1, 1] * 4
check_false online-dfa-reversed 2 test/01.spec test/01-08.in # [1, 0] * 4
check_true  online-dfa-reversed 2 test/01.spec test/01-01.in # [0, 0] * 8 * 100
check_false online-dfa-reversed 2 test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  online-dfa-reversed 2 test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20
check_true  online-dfa-reversed-with-rev-spec 2 test/01_rev.spec test/01-07.in
check_false online-dfa-reversed-with-rev-spec 2 test/01_rev.spec test/01-08.in
check_true  online-dfa-reversed 9 test/10.spec test/10-01.in # "111111111" * 100
check_false online-dfa-reversed 9 test/10.spec test/10-02.in # "111111110" * 100
check_true  online-dfa-reversed 9 test/10.spec test/10-03.in # "111111110" * 90

#### Online DFA (qtrlwe2)
check_true  online-dfa-qtrlwe2 2 test/01.spec test/01-07.in # [1, 1] * 4
check_false online-dfa-qtrlwe2 2 test/01.spec test/01-08.in # [1, 0] * 4
check_true  online-dfa-qtrlwe2 2 test/01.spec test/01-01.in # [0, 0] * 8 * 100
check_false online-dfa-qtrlwe2 2 test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  online-dfa-qtrlwe2 2 test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20
check_true  online-dfa-qtrlwe2 9 test/10.spec test/10-01.in # "111111111" * 100
check_false online-dfa-qtrlwe2 9 test/10.spec test/10-02.in # "111111110" * 100
check_true  online-dfa-qtrlwe2 9 test/10.spec test/10-03.in # "111111110" * 90

#### Online DFA (block-backstream)
check_true  online-dfa-blockbackstream 2 test/01.spec test/01-07.in # [1, 1] * 4
check_false online-dfa-blockbackstream 2 test/01.spec test/01-08.in # [1, 0] * 4
check_true  online-dfa-blockbackstream 2 test/01.spec test/01-01.in # [0, 0] * 8 * 100
check_false online-dfa-blockbackstream 2 test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  online-dfa-blockbackstream 2 test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20
check_true  online-dfa-blockbackstream 9 test/10.spec test/10-01.in # "111111111" * 100
check_false online-dfa-blockbackstream 9 test/10.spec test/10-02.in # "111111110" * 100
check_true  online-dfa-blockbackstream 9 test/10.spec test/10-03.in # "111111110" * 90

### Clean up temporary files
#rm _test_sk _test_bk _test_in _test_out #_test_random.log

