#!/usr/bin/env bash
# Configure, build and run the host tests for the portable sources in main/.
# The firmware itself still builds with idf.py; this covers only the code that
# has no ESP-IDF dependency (the device root secret KDF and the ETS device
# certificate encoding).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-${repo_root}/build_host_tests}"

cmake -S "${repo_root}/main/test" -B "${build_dir}"
cmake --build "${build_dir}" -j"$(nproc)"
ctest --test-dir "${build_dir}" --output-on-failure
