#pragma once

#include "common/types.h"

#include <string>

namespace ps2 {

class EeBus;
class GsCore;

class GifDma {
public:
    void reset();
    [[nodiscard]] bool service(EeBus& bus, GsCore& gs, std::string& error);

private:
    [[nodiscard]] bool complete(EeBus& bus, u32 chcr);
    bool end_after_qwc_ = false;
};

} // namespace ps2
