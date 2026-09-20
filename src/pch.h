#pragma once

// Stable standard-library precompiled header for VibeStation.
//
// Keep project headers and frequently changed third-party/platform headers out
// of here. The goal is to avoid reparsing the C++ standard library in every
// translation unit without making small emulator-header edits invalidate the
// entire build.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
