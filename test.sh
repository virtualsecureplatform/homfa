#!/usr/bin/bash

TEST0=build/bin/test0
HOMFA=build/bin/homfa

failwith(){
    echo -ne "\e[1;31m[ERROR]\e[0m "
    echo "$1"
    exit 1
}

enc_run_dec(){
    $HOMFA enc --key _test_sk --in "$3" --out _test_in
    case "$1" in
        "offline-dfa" )
            $HOMFA run-offline-dfa --bkey _test_bk --spec "$2" --in _test_in --out _test_out
            ;;
        "online-dfa" )
            $HOMFA run-online-dfa --bkey _test_bk --spec "$2" --in _test_in --out _test_out
            ;;
        * )
            failwith "Invalid run $1"
            ;;
    esac
    $HOMFA dec --key _test_sk --in _test_out
}

check_true(){
    res=$(enc_run_dec "$1" "$2" "$3")
    echo "$res" | grep "Result (bool): true" > /dev/null
    [ $? -eq 0 ] || failwith "Expected true, got false >>> enc_run_dec \"$1\" \"$2\" \"$3\"\\$res"
}

check_false(){
    res=$(enc_run_dec "$1" "$2" "$3")
    echo "$res" | grep "Result (bool): false" > /dev/null
    [ $? -eq 0 ] || failwith "Expected false, got true >>> enc_run_dec \"$1\" \"$2\" \"$3\"\\$res"
}

### Prepare secret key and bootstrapping key
[ -f _test_sk ] || $HOMFA genkey --out _test_sk
[ -f _test_bk ] || $HOMFA genbkey --key _test_sk --out _test_bk

### Now start testing
$TEST0
#### Offline DFA
check_true  offline-dfa test/01.spec test/01-01.in # [1, 1] * 8 * 100
check_false offline-dfa test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  offline-dfa test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20
check_false offline-dfa test/01.spec test/01-04.in # [1, 0] + ([0, 0, 1, 0] * 8 * 20) + [0, 1] + ([0, 0, 0, 1] * 8 * 20)
check_true  offline-dfa test/01.spec test/01-05.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 100
check_false offline-dfa test/01.spec test/01-06.in # [1, 0] + ([0, 0, 1, 0] * 8 * 100) + [0, 1] + ([0, 0, 0, 1] * 8 * 150)

#### Online DFA
check_true  online-dfa test/01.spec test/01-01.in # [1, 1] * 8 * 100
check_false online-dfa test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  online-dfa test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20

### Clean up temporary files
rm _test_sk _test_bk _test_in _test_out
