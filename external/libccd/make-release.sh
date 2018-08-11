#!/bin/bash

# Creates .tar.gz package of specified version.
# Takes one argument - identification of commit

NAME=libccd
COMMIT=""
CMD="git archive"

# read arguments
COMMIT="$1"

if [ "$COMMIT" = "" ]; then
    echo "Usage: $0 commit [--notest] [--nodoc]"
    echo "Error: you must specify commit which should be packed"
    exit -1;
fi;


PREFIX=${NAME}-$COMMIT/
FN=${NAME}-$COMMIT.tar.gz

if echo "$COMMIT" | grep '^v[0-9]\.[0-9]\+' >/dev/null 2>&1; then
    tmp=$(echo "$COMMIT" | sed 's/^v//')
    PREFIX=${NAME}-$tmp/
    FN=${NAME}-$tmp.tar.gz
fi

$CMD --prefix="$PREFIX" --format=tar $COMMIT | gzip >"$FN"
echo "Package: $FN"

