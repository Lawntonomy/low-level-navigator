#! /bin/bash
# Configure, build, and run the host-side unit tests. No Pico SDK or ARM
# toolchain involved -- this builds with the host compiler and runs locally.
#
# The first run downloads googletest via CMake FetchContent and needs network
# access; later runs use the copy cached in build-test/.

set -euo pipefail

cd "$(dirname "$0")/.."

cmake -S test -B build-test -DCMAKE_BUILD_TYPE=Debug
cmake --build build-test -j"$(nproc)"

ctest --test-dir build-test --output-on-failure "$@"
