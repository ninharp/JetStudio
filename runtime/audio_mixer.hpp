#pragma once
#include <atomic>
#include <cstddef>
#include <cmath>
#include <cstdint>

// Mixes up to four sounds into a 16 bit mono stream. One thread (the game) calls play, stop and is_playing,
// another (the audio task) calls mix; they share only the command ring and the finished serials, so neither
// ever waits for the other.
namespace jetstudio {
constexpr int AudioSampleRate = 22050, AudioVoices = 4;
constexpr size_t MaxSoundBytes = 2 * 1024 * 1024; // a converted sound, 16 bit mono: about 47 seconds

class AudioMixer {
public:
    void set_max_db(float db) {
        max_db_ = db;
        limit_ = int32_t(std::lround(32767.0 * std::pow(10.0, db / 20.0)));
    } // the board's limit, before the first play()
    // samples: little endian 16 bit, unaligned is fine. false when the ring has no room (the last slots are kept for stops); the sound is dropped.
    // A playing owner restarts in its voice; with none free the oldest voice is replaced.
    // The samples must stay valid until after the next mix() following a stop, restart or replacement,
    // because the audio task reads them until then.
    bool play(uint32_t owner, const uint8_t* samples, uint32_t frames, float volume_db, bool loop);
    void stop(uint32_t owner);
    void stop_all();
    bool is_playing(uint32_t owner) const;
    void mix(int16_t* out, size_t frames);

private:
    struct Command {
        const uint8_t* samples; // null: stop
        uint32_t frames, serial;
        int32_t gain; // 4.12 fixed point
        uint8_t voice;
        bool loop;
    };
    static constexpr uint32_t RingSize = 32;
    bool push(const Command& c, uint32_t keep_free = 0);

    // Game side
    struct Slot { uint32_t owner = 0, serial = 0; };
    Slot slots_[AudioVoices];
    uint32_t next_serial_ = 0;
    float max_db_ = 0;
    int32_t limit_ = 32767; // the board limit also caps the sum

    // Shared
    Command ring_[RingSize];
    std::atomic<uint32_t> head_{0}, tail_{0}; // head: written by the game, tail: by the audio task
    std::atomic<uint32_t> finished_[AudioVoices] = {};

    // Audio side
    struct Voice {
        const uint8_t* samples = nullptr;
        uint32_t frames = 0, position = 0, serial = 0;
        int32_t gain = 0;
        bool loop = false;
    };
    Voice voices_[AudioVoices];
};
} // namespace jetstudio
