#pragma once

#include "common/types.h"

#include <string>

namespace ps2 {

class EeBus;
class GsCore;
class GsPrivileged;

class Vif1Dma {
public:
    void reset() {}

    [[nodiscard]] bool service(
        EeBus& bus,
        GsCore& gs,
        const GsPrivileged& privileged,
        std::string& error);

private:
    [[nodiscard]] bool complete(EeBus& bus, u32 chcr);
};

} // namespace ps2
