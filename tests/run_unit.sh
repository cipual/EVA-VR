#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_output="${1:?Specify a debug output directory}"
mkdir -p "$test_output"
"${CXX:-c++}" -std=c++14 -Wall -Wextra -Werror \
  -Iapp/src/main/cpp -Iapp/src/main/cpp/openxr_loader/include -Iapp/src/main/cpp/third \
  tests/input_reference_space_test.cpp -o "$test_output/input_reference_space_test"
"$test_output/input_reference_space_test" "$test_output/controller_direction_cases.jsonl"
