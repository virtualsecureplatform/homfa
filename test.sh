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

check_true test/03.spec test/03.in
