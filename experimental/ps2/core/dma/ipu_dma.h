#pragma once

#include "common/types.h"
#include <string>

namespace ps2 {
class EeBus;

class IpuDma {
public:
    void reset();
    bool service(EeBus& bus, std::string& error);

private:
    bool end_to_ipu_ = false;

    bool service_from_ipu(EeBus& bus, std::string& error);
    bool service_to_ipu(EeBus& bus, std::string& error);
};
} // namespace ps2
