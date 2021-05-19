#!/usr/bin/bash

HOMFA=build/bin/homfa

failwith(){
    echo -ne "\e[1;31m[ERROR]\e[0m "
    echo "$1"
    exit 1
}

check_true(){
    $HOMFA "$1" "$2" | grep "Result (bool): true" > /dev/null
    [ $? -eq 0 ] || failwith "Expected true, got false >>> $HOMFA \"$1\" \"$2\""
}

check_false(){
    $HOMFA "$1" "$2" | grep "Result (bool): false" > /dev/null
    [ $? -eq 0 ] || failwith "Expected false, got true >>> $HOMFA \"$1\" \"$2\""
}

check_true  test/01.spec test/01-01.in # [1, 1] * 8 * 100
check_false test/01.spec test/01-02.in # [1, 0] * 8 * 100
check_true  test/01.spec test/01-03.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 20
check_false test/01.spec test/01-04.in # [1, 0] + ([0, 0, 1, 0] * 8 * 20) + [0, 1] + ([0, 0, 0, 1] * 8 * 20)
check_true  test/01.spec test/01-05.in # [0, 0, 1, 0, 0, 1, 0, 1, 1, 1] * 8 * 100
check_false test/01.spec test/01-06.in # [1, 0] + ([0, 0, 1, 0] * 8 * 100) + [0, 1] + ([0, 0, 0, 1] * 8 * 150)
