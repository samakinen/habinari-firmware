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
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

# name : extra defaults files, semicolon separated (empty = the plain default,
# KNX TP1 + Modbus). sdkconfig.defaults.ble is an overlay rather than a variant:
# the BLE service channel is not a personality, it composes with one. There is
# deliberately no knx-ble — Kconfig refuses it, and oob_service.h says why.
declare -A variants=(
    [knx]=""
    [modbus]="sdkconfig.defaults.modbus"
    [modbus-ble]="sdkconfig.defaults.modbus;sdkconfig.defaults.ble"
    [mqtt]="sdkconfig.defaults.mqtt;sdkconfig.defaults.ble"
)

selected=("$@")
if [ ${#selected[@]} -eq 0 ]; then
    selected=(knx modbus modbus-ble mqtt)
fi

for name in "${selected[@]}"; do
    if [ -z "${variants[$name]+set}" ]; then
        echo "unknown variant '${name}' (have: ${!variants[*]})" >&2
        exit 2
    fi

    build_dir="build_${name}"
    defaults="sdkconfig.defaults"
    if [ -n "${variants[$name]}" ]; then
        defaults="sdkconfig.defaults;${variants[$name]}"
    fi

    echo "=== ${name} -> ${build_dir} ==============================="
    # The ETS export only means anything for a KNX image, and regenerating it
    # from a build that has no KNX application would be misleading.
    ets_export=ON
    if [ "${name}" != "knx" ]; then
        ets_export=OFF
    fi

    idf.py -B "${build_dir}" \
           -DSDKCONFIG="${build_dir}/sdkconfig" \
           -DSDKCONFIG_DEFAULTS="${defaults}" \
           -DHABINARI_ETS_EXPORT="${ets_export}" \
           build

    grep -E "^CONFIG_HABINARI_(PROTOCOL_[A-Z]+|OOB_BLE)=y" "${build_dir}/sdkconfig" \
        || echo "  (no personality selected)"
done

echo
echo "=== image sizes ============================================="
for name in "${selected[@]}"; do
    bin="build_${name}/habinari.bin"
    if [ -f "${bin}" ]; then
        printf '%-8s %8s bytes\n' "${name}" "$(stat -c%s "${bin}")"
    fi
done
