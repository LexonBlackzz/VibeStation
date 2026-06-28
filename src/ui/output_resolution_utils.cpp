#include "ui/output_resolution_utils.h"

void output_resolution_dimensions(OutputResolutionMode mode,
    int& out_width, int& out_height) {
    switch (mode) {
    case OutputResolutionMode::R640x480:
        out_width = 640;
        out_height = 480;
        break;
    case OutputResolutionMode::R1024x768:
        out_width = 1024;
        out_height = 768;
        break;
    case OutputResolutionMode::R320x240:
    default:
        out_width = 320;
        out_height = 240;
        break;
    }
}
