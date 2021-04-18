#!/bin/sh
set -eu


# CRETING UNIT TEST BUILD DIRECTORY AND CD IN TO IT
#cd -- "$(dirname -- "$0")"
mkdir -p buildUT
cd buildUT

# CREATE THE CMAKE CACHE IN BUILD DIRECTORY
cmake ../test

# BUILD USING MAKE
make

# RUN TEST
ctest -VV -debug
