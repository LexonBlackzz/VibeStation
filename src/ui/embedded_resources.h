#pragma once

#include <cstddef>

namespace vibestation {

struct EmbeddedResourceView {
    const unsigned char* data = nullptr;
    std::size_t size = 0;

    explicit operator bool() const {
        return data != nullptr && size != 0;
    }
};

// Returns a non-owning view into the executable's Windows RCDATA section.
// On non-Windows builds, or when the resource is unavailable, returns empty.
EmbeddedResourceView embedded_resource(int resource_id);

} // namespace vibestation
