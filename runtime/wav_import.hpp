#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace jetstudio {
// A PCM WAV (8 or 16 bit, 1 or 2 channels, any rate) as little endian 16 bit mono at AudioSampleRate.
bool decode_wav(const uint8_t* data, size_t size, std::vector<uint8_t>& out, std::string& error);
}
