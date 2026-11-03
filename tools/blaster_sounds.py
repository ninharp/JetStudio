#!/usr/bin/env python3
"""Writes Jet Blaster's arcade sounds into <project>/sounds. Square waves and noise, standard library only."""
import argparse
import random
import struct
import wave
from pathlib import Path

RATE = 22050


def write(path, samples):
    with wave.open(str(path), "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(RATE)
        out.writeframes(b"".join(struct.pack("<h", int(max(-1.0, min(1.0, s)) * 26000)) for s in samples))


def sweep(start, end, seconds, volume=0.5):
    """A square wave gliding from start to end Hz, fading out."""
    phase = 0.0
    count = int(RATE * seconds)
    for i in range(count):
        t = i / count
        phase = (phase + (start + (end - start) * t) / RATE) % 1.0
        yield (volume if phase < 0.5 else -volume) * (1.0 - t)


def noise(seconds, hold, volume=0.8):
    """Sample-and-hold noise fading out; a longer hold sounds deeper."""
    rng = random.Random(1)
    value = 0.0
    count = int(RATE * seconds)
    for i in range(count):
        if i % hold == 0:
            value = rng.uniform(-1.0, 1.0)
        yield value * volume * (1.0 - i / count)


SOUNDS = {
    "shot.wav": lambda: sweep(1400, 500, 0.06, 0.3), # fires every 0.16 s: short and quiet
    "hit.wav": lambda: list(sweep(300, 120, 0.12)) + list(noise(0.08, 6, 0.4)),
    "explosion.wav": lambda: noise(0.35, 8),
    "game_over.wav": lambda: list(sweep(440, 330, 0.25)) + list(sweep(330, 220, 0.25)) + list(sweep(220, 110, 0.5)),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    project = parser.parse_args().project
    (project / "sounds").mkdir(exist_ok=True)
    for name, make in SOUNDS.items():
        write(project / "sounds" / name, make())
        print(project / "sounds" / name)


if __name__ == "__main__":
    main()
