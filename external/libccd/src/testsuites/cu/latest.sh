#!/bin/bash

repo=http://git.danfis.cz/cu.git
files="COPYING \
       COPYING.LESSER \
       cu.h \
       cu.c \
       Makefile \
       check-regressions \
       .gitignore \
       "
rm -rf cu
git clone $repo
for file in $files; do
    mv cu/"$file" .
done;
rm -rf cu
