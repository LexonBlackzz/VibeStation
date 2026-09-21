#pragma once

#include "common/types.h"

#include <string>
#include <vector>

namespace ps2 {

class EeBus;
class IopBus;
class IopIntc;

class SifDma {
public:
    void reset();

    [[nodiscard]] bool service(
        EeBus& ee_bus,
        IopBus& iop_bus,
        IopIntc& iop_intc,
        std::string& error);

private:
    [[nodiscard]] bool service_sif0(
        EeBus& ee_bus,
        IopBus& iop_bus,
        IopIntc& iop_intc,
        std::string& error);
    [[nodiscard]] bool service_sif1(
        EeBus& ee_bus,
        IopBus& iop_bus,
        IopIntc& iop_intc,
        std::string& error);

    [[nodiscard]] bool collect_ee_source_chain(
        EeBus& bus,
        u32 channel_base,
        std::vector<u32>& words,
        u32& final_chcr,
        std::string& error);

    static void append_qword(
        std::vector<u32>& words,
        u64 lo,
        u64 hi);
};

} // namespace ps2
