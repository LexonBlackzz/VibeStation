#pragma once

#include "core/types.h"
#include <vector>

void resample_rgba_nearest(const std::vector<u32>& src, int src_width, int src_height,
    std::vector<u32>& dst, int dst_width, int dst_height);
bool encode_rgba_png(const std::vector<u32>& rgba, int width, int height,
    std::vector<u8>& out_png);
