#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Sami Mäkinen
# Build every field-bus personality.
#
# Each variant gets its own build directory AND its own sdkconfig. The second
# half matters: idf.py defaults to the project-root ./sdkconfig regardless of
# -B, so building two variants without -DSDKCONFIG makes the second one silently
# inherit the first one's configuration — the symptom is two "different"
# variants coming out the same size.
#
#   tools/build_variants.sh            build all variants
#   tools/build_variants.sh mqtt       build one
#   tools/build_variants.sh --prod     release-trim them: sdkconfig.defaults.prod
#                                      is layered last, into build_<name>_prod
#
# A KNX image also builds its ETS product (README, "ETS product"), which needs
# a C++23 host compiler next to the ESP-IDF toolchain; KNX_ETS_PRODUCT=OFF in
# the environment skips it.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

# name : extra defaults files, semicolon separated (empty = the plain default,
# KNX TP1 + Modbus). sdkconfig.defaults.ble is an overlay rather than a variant:
# the BLE service channel is not a personality, it composes with one. There is
# deliberately no knx-ble — Kconfig refuses it, and oob_service.h says why.
declare -A variants=(
    [knx]=""
    [knxip]="sdkconfig.defaults.knxip;sdkconfig.defaults.ble"
    [modbus]="sdkconfig.defaults.modbus"
    [modbus-ble]="sdkconfig.defaults.modbus;sdkconfig.defaults.ble"
    [mqtt]="sdkconfig.defaults.mqtt;sdkconfig.defaults.ble"
)

prod=0
selected=()
for arg in "$@"; do
    case "${arg}" in
        --prod) prod=1 ;;
        *) selected+=("${arg}") ;;
    esac
done
if [ ${#selected[@]} -eq 0 ]; then
    selected=(knx knxip modbus modbus-ble mqtt)
fi

build_suffix=""
if [ "${prod}" -eq 1 ]; then
    build_suffix="_prod"
fi

for name in "${selected[@]}"; do
    if [ -z "${variants[$name]+set}" ]; then
        echo "unknown variant '${name}' (have: ${!variants[*]})" >&2
        exit 2
    fi

    build_dir="build_${name}${build_suffix}"
    defaults="sdkconfig.defaults"
    if [ -n "${variants[$name]}" ]; then
        defaults="sdkconfig.defaults;${variants[$name]}"
    fi
    if [ "${prod}" -eq 1 ]; then
        defaults="${defaults};sdkconfig.defaults.prod"
    fi

    echo "=== ${name} -> ${build_dir} ==============================="
    # A KNX image builds its ETS product; the others have none.
    idf.py -B "${build_dir}" \
           -DSDKCONFIG="${build_dir}/sdkconfig" \
           -DSDKCONFIG_DEFAULTS="${defaults}" \
           -DKNX_ETS_PRODUCT="${KNX_ETS_PRODUCT:-ON}" \
           build

    grep -E "^CONFIG_HABINARI_(PROTOCOL_[A-Z]+|KNX_MEDIUM_[A-Z0-9]+|OOB_BLE)=y" "${build_dir}/sdkconfig" \
        || echo "  (no personality selected)"
done

echo
echo "=== image sizes ============================================="
for name in "${selected[@]}"; do
    bin="build_${name}${build_suffix}/habinari.bin"
    if [ -f "${bin}" ]; then
        printf '%-16s %8s bytes\n' "${name}${build_suffix}" "$(stat -c%s "${bin}")"
    fi
done
