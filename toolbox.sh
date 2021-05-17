#!/bin/bash -xe

case "$1" in
    bump-submodule )
        [ $# -eq 2 ] || ( echo "Usage: $0 bump-submodule SUBMODULE-DIR"; exit 1 )
        git reset
        diff=$(git diff "$2" | grep Subproject | cut -f3 -d' ' | (read l1; read l2; echo -e "$l1\t$l2"))
        echo "$diff" | egrep "^[a-f0-9]{40}	[a-f0-9]{40}$" > /dev/null || ( echo "Invalid diff"; exit 1 )
        git add "$2" && git commit -m \
            "$(echo "$diff" | awk "{print \"Bump $2\",\"from\",\$1,\"to\",\$2}")"
        ;;

    * )
        echo "Usage: $0 bump-submodule"
esac
