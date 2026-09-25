#include "ui/embedded_resources.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace vibestation {

EmbeddedResourceView embedded_resource(int resource_id) {
#ifdef _WIN32
    HMODULE module = GetModuleHandleW(nullptr);
    if (module == nullptr) {
        return {};
    }

    HRSRC resource = FindResourceW(
        module,
        MAKEINTRESOURCEW(resource_id),
        MAKEINTRESOURCEW(10));
    if (resource == nullptr) {
        return {};
    }

    const DWORD byte_count =
        SizeofResource(module, resource);
    if (byte_count == 0) {
        return {};
    }

    HGLOBAL loaded =
        LoadResource(module, resource);
    if (loaded == nullptr) {
        return {};
    }

    const void* bytes =
        LockResource(loaded);
    if (bytes == nullptr) {
        return {};
    }

    return {
        static_cast<const unsigned char*>(bytes),
        static_cast<std::size_t>(byte_count)
    };
#else
    (void)resource_id;
    return {};
#endif
}

} // namespace vibestation
