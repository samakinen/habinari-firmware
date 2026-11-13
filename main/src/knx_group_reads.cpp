// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Group-object read responses. Every transmitting port answers an
// A_GroupValue_Read with exactly the value publication would send for the same
// state — the corrected reading, from the same function (knx_port_values.hpp),
// so a bus read and a spontaneous send can never disagree. With no valid
// reading, where publication sends nothing, a read is answered with the
// port's "no reading" form: DPT 9 7FFFh "invalid data" for the measurements.

#include "knx_service_internal.hpp"

#include "knx_port_values.hpp"
#include "knx_product.hpp"

namespace habinari_knx::service {

void bindGroupObjectReads(HabinariBindings &bindings)
{
    forEachTransmitPort([&bindings]<HabinariPort Port>(const char *) {
        bindings.template provideState<Port>(
            []() { return readResponse<Port>(currentPortValues()); });
    });
}

}  // namespace habinari_knx::service
