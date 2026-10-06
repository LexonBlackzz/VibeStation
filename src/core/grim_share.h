#pragma once
#include "grim_genome.h"
#include <string>

// Share codes ("corruption codes"): a genome as one line of text to paste anywhere.
// "VSGRIM1:" + base64url(zlib(genome JSON)). The JSON stays the genome format; the
// code is only a compact transport for it.
std::string grim_share_code(const GrimGenome &genome);

// Reads a share code, or plain genome JSON (what Copy code produced before codes).
// Surrounding whitespace and line breaks inside the code are ignored.
bool grim_share_parse(const std::string &text, GrimGenome &out, std::string &err);
