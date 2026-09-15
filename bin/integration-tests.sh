#!/usr/bin/env bash
#
# Build and run the software-device integration tests (ctest label
# "integration", test/integration_tests.cpp). No camera needed. Expects a
# configured build tree; see "Build and test" in the README.
#
#   ./bin/integration-tests.sh                 # all
#   ./bin/integration-tests.sh -R PointCloud   # extra args go to ctest
#   BUILD_DIR=build-asan/build/Release ./bin/integration-tests.sh

set -euxo pipefail

cd "$(dirname "$0")/.."

BUILD_DIR="${BUILD_DIR:-build/Release}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN)}"

# Every test binary, not just integration_tests: ctest discovers gtest cases
# from all of them at test time.
cmake --build "${BUILD_DIR}" -j "${JOBS}"

# Each case is its own process; -j overlaps the per-process teardown.
ctest --test-dir "${BUILD_DIR}" -L integration -j "${JOBS}" --output-on-failure "$@"
