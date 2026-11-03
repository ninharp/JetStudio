#include "audio_mixer.hpp"
#include <algorithm>
#include <cmath>

namespace jetstudio {

bool AudioMixer::push(const Command& c, uint32_t keep_free) {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= RingSize - keep_free) return false;
    ring_[head % RingSize] = c;
    head_.store(head + 1, std::memory_order_release);
    return true;
}

bool AudioMixer::play(uint32_t owner, const uint8_t* samples, uint32_t frames, float volume_db, bool loop) {
    if (!samples || !frames) return false;
    int voice = -1;
    for (int i = 0; i < AudioVoices && voice < 0; ++i) if (slots_[i].owner == owner) voice = i;
    for (int i = 0; i < AudioVoices && voice < 0; ++i)
        if (slots_[i].serial == finished_[i].load(std::memory_order_acquire) || !slots_[i].owner) voice = i;
    if (voice < 0) {
        voice = 0;
        for (int i = 1; i < AudioVoices; ++i) if (slots_[i].serial - slots_[voice].serial > 0x80000000u) voice = i;
    }
    const uint32_t serial = ++next_serial_;
    const auto gain = int32_t(std::lround(std::pow(10.0f, (std::min(volume_db, 0.0f) + max_db_) / 20.0f) * 4096.0f));
    // Keep one ring slot per voice for the stops: a stop must never be lost, or its voice would play on.
    if (!push({samples, frames, serial, gain, uint8_t(voice), loop}, AudioVoices)) return false;
    slots_[voice] = {owner, serial};
    return true;
}

void AudioMixer::stop(uint32_t owner) {
    for (int i = 0; i < AudioVoices; ++i)
        if (slots_[i].owner == owner) {
            push({nullptr, 0, 0, 0, uint8_t(i), false});
            slots_[i].owner = 0;
        }
}

void AudioMixer::stop_all() {
    for (int i = 0; i < AudioVoices; ++i)
        if (slots_[i].owner) {
            push({nullptr, 0, 0, 0, uint8_t(i), false});
            slots_[i].owner = 0;
        }
}

bool AudioMixer::is_playing(uint32_t owner) const {
    for (int i = 0; i < AudioVoices; ++i)
        if (owner && slots_[i].owner == owner && slots_[i].serial != finished_[i].load(std::memory_order_acquire)) return true;
    return false;
}

void AudioMixer::mix(int16_t* out, size_t frames) {
    uint32_t tail = tail_.load(std::memory_order_relaxed);
    const uint32_t head = head_.load(std::memory_order_acquire);
    for (; tail != head; ++tail) {
        const Command& c = ring_[tail % RingSize];
        Voice& v = voices_[c.voice];
        v.samples = c.samples;
        if (c.samples) {
            v.frames = c.frames; v.position = 0; v.serial = c.serial; v.gain = c.gain; v.loop = c.loop;
        }
    }
    tail_.store(tail, std::memory_order_release);

    for (size_t i = 0; i < frames; ++i) {
        int32_t sum = 0;
        for (int k = 0; k < AudioVoices; ++k) {
            Voice& v = voices_[k];
            if (!v.samples) continue;
            const uint8_t* p = v.samples + size_t(v.position) * 2;
            sum += (int32_t(int16_t(uint16_t(p[0]) | uint16_t(p[1]) << 8)) * v.gain) >> 12;
            if (++v.position == v.frames) {
                if (v.loop) v.position = 0;
                else {
                    v.samples = nullptr;
                    finished_[k].store(v.serial, std::memory_order_release);
                }
            }
        }
        out[i] = int16_t(std::clamp<int32_t>(sum, limit_ == 32767 ? -32768 : -limit_, limit_));
    }
}

} // namespace jetstudio
