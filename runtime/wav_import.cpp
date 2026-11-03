#include "wav_import.hpp"
#include "audio_mixer.hpp"
#include <cstring>

namespace jetstudio {
namespace {
uint32_t u32_at(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
uint16_t u16_at(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }
}

bool decode_wav(const uint8_t* data, size_t size, std::vector<uint8_t>& out, std::string& error) {
    out.clear();
    if (!data || size < 12 || std::memcmp(data, "RIFF", 4) || std::memcmp(data + 8, "WAVE", 4)) { error = "not a WAV file"; return false; }
    int format = 0, channels = 0, bits = 0;
    uint32_t rate = 0;
    const uint8_t* pcm = nullptr;
    size_t pcm_bytes = 0;
    for (size_t at = 12; at + 8 <= size;) {
        const uint32_t length = u32_at(data + at + 4);
        const uint8_t* body = data + at + 8;
        const size_t available = size - at - 8;
        if (!std::memcmp(data + at, "fmt ", 4)) {
            if (length < 16 || available < 16) { error = "truncated WAV"; return false; }
            format = u16_at(body); channels = u16_at(body + 2); rate = u32_at(body + 4); bits = u16_at(body + 14);
            // WAVE_FORMAT_EXTENSIBLE carries the real format at the start of its GUID.
            if (format == 0xfffe && length >= 26 && available >= 26) format = u16_at(body + 24);
        } else if (!std::memcmp(data + at, "data", 4)) {
            if (length > available) { error = "truncated WAV"; return false; }
            pcm = body; pcm_bytes = length;
            break;
        }
        if (length > available) { error = "truncated WAV"; return false; }
        at += 8 + size_t(length) + (length & 1);
    }
    if (!format || !pcm) { error = "truncated WAV"; return false; }
    if (format != 1 || (bits != 8 && bits != 16)) { error = "only PCM WAV with 8 or 16 bits is supported"; return false; }
    if ((channels != 1 && channels != 2) || !rate) { error = "only mono or stereo WAV is supported"; return false; }
    const size_t frame_bytes = size_t(channels) * bits / 8, frames = pcm_bytes / frame_bytes;
    if (!frames) { error = "empty WAV"; return false; }
    const auto mono = [&](size_t i) -> int32_t {
        const uint8_t* p = pcm + i * frame_bytes;
        int32_t sum = 0;
        for (int c = 0; c < channels; ++c)
            sum += bits == 8 ? (int32_t(p[c]) - 128) * 256 : int32_t(int16_t(u16_at(p + c * 2)));
        return sum / channels;
    };
    // 64 bit here: a tiny file with a tiny rate would otherwise ask for gigabytes.
    const uint64_t length = rate == uint32_t(AudioSampleRate) ? frames : (uint64_t(frames - 1) * AudioSampleRate) / rate + 1;
    if (length * 2 > MaxSoundBytes) { error = "WAV too long: at most 2 MB after conversion (about 47 seconds)"; return false; }
    out.reserve(size_t(length * 2));
    for (size_t i = 0; i < length; ++i) {
        const uint64_t position = uint64_t(i) * rate; // in 1/AudioSampleRate source frames
        const size_t index = size_t(position / AudioSampleRate);
        const int32_t fraction = int32_t(position % AudioSampleRate);
        const int32_t a = mono(index), b = index + 1 < frames ? mono(index + 1) : a;
        const auto s = int16_t(a + int64_t(b - a) * fraction / AudioSampleRate);
        out.push_back(uint8_t(s));
        out.push_back(uint8_t(uint16_t(s) >> 8));
    }
    return true;
}
} // namespace jetstudio
