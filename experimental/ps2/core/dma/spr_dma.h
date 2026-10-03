#pragma once

#include "common/types.h"
#include <string>

namespace ps2 {
class EeBus;

class SprDma {
public:
    void reset();
    bool service(EeBus& bus, std::string& error);

private:
    struct ChannelState { bool end_after_qwc = false; };

    bool service_from_spr(EeBus& bus, std::string& error);
    bool service_to_spr(EeBus& bus, std::string& error);
    bool transfer_from_spr(
        EeBus& bus, u32& madr, u32& qwc, u32& sadr,
        std::string& error);
    bool transfer_to_spr(
        EeBus& bus, u32& madr, u32& qwc, u32& sadr,
        std::string& error);

    ChannelState from_{};
    ChannelState to_{};
};
} // namespace ps2
