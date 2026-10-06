#pragma once

// Which side VibeStation opens in: VibeStation 1 (PS1) or VibeStation 2
// (PS2). Both apps' settings change it and main() reads it before either
// starts, so it lives in its own small file next to their configs
// (vibestation_config.ini, vibestation2.ini). Header-only and C++17, because
// the main executable and the PS2 code build with different standards.

#include <fstream>
#include <string>

namespace vibestation {

enum class FavoriteEmulator { VibeStation1, VibeStation2 };

inline constexpr const char* kFavoriteEmulatorFile = "vibestation_favorite.ini";

namespace detail {
// Read once per run; both settings screens ask every frame they are shown.
struct FavoriteCache {
    bool loaded = false;
    FavoriteEmulator value = FavoriteEmulator::VibeStation1;
};
inline FavoriteCache& favorite_cache() {
    static FavoriteCache cache;
    return cache;
}
} // namespace detail

inline FavoriteEmulator load_favorite_emulator() {
    detail::FavoriteCache& cache = detail::favorite_cache();
    if (!cache.loaded) {
        cache.loaded = true;
        std::ifstream file(kFavoriteEmulatorFile);
        std::string line;
        while (std::getline(file, line)) {
            if (line.rfind("favorite=", 0) == 0) {
                cache.value = line.compare(9, 3, "vs2") == 0 ? FavoriteEmulator::VibeStation2
                                                              : FavoriteEmulator::VibeStation1;
            }
        }
    }
    return cache.value;
}

inline void save_favorite_emulator(FavoriteEmulator favorite) {
    detail::FavoriteCache& cache = detail::favorite_cache();
    cache.loaded = true;
    cache.value = favorite;
    std::ofstream file(kFavoriteEmulatorFile, std::ios::trunc);
    file << "favorite=" << (favorite == FavoriteEmulator::VibeStation2 ? "vs2" : "vs1") << '\n';
}

} // namespace vibestation
