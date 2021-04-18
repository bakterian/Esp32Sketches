#!/bin/sh
set -eu

# BUILD DIRECTORY (current dir) 
# binary files can be found ing: build.esp32

# CREATE THE CMAKE CACHE IN BUILD DIRECTORY (CONFIGRE AND GENERATE)
cmake .

# BUILD USING A CMAKE CUSTOM COMMAND
cmake --build . --target AppBuild