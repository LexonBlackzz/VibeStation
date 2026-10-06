#pragma once

// The non-affiliation notice VibeStation 1 and VibeStation 2 show before
// they start. It stays up until "I understand"; with "Don't show this
// disclaimer again" ticked (the default) it is never shown again, by either
// app. Confirming it once also covers the other app for the rest of the run.
// Header-only and C++17, like favorite_emulator.h, because the main
// executable and the PS2 code build with different standards.

#include <fstream>
#include <string>

namespace vibestation {

inline constexpr const char* kStartupDisclaimerFile = "vibestation_disclaimer.ini";

namespace detail {
struct DisclaimerState {
    bool loaded = false;
    bool remembered = false;    // ticked once: never again
    bool seen_this_run = false; // confirmed in this run, by either app
};
inline DisclaimerState& disclaimer_state() {
    static DisclaimerState state;
    return state;
}
} // namespace detail

inline bool startup_disclaimer_needed() {
    detail::DisclaimerState& state = detail::disclaimer_state();
    if (!state.loaded) {
        state.loaded = true;
        std::ifstream file(kStartupDisclaimerFile);
        std::string line;
        while (std::getline(file, line)) {
            if (line.rfind("acknowledged=1", 0) == 0) state.remembered = true;
        }
    }
    return !state.remembered && !state.seen_this_run;
}

// "I understand" was pressed; `remember` is the checkbox.
inline void acknowledge_startup_disclaimer(bool remember) {
    detail::DisclaimerState& state = detail::disclaimer_state();
    state.seen_this_run = true;
    if (remember) {
        state.remembered = true;
        std::ofstream file(kStartupDisclaimerFile, std::ios::trunc);
        file << "acknowledged=1\n";
    }
}

} // namespace vibestation
