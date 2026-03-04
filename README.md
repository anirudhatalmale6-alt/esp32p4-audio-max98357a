# ESP32-P4 Audio Player with MAX98357A

Plays embedded WAV files through a MAX98357A I2S amplifier using ESP-IDF 5.5.3.

## Hardware

| Signal | GPIO | MAX98357A Pin |
|--------|------|---------------|
| BCLK   | 14   | BCLK          |
| LRC    | 15   | LRC           |
| DOUT   | 13   | DIN           |

## Quick Start

```bash
# Set target (only needed once)
idf.py set-target esp32p4

# Build and flash
idf.py build flash monitor
```

## Adding Audio Files

1. Place `.wav` files in `main/audio/`
2. Build and flash — files are automatically embedded into the firmware

### Accessing embedded files in code

For a file at `main/audio/myfile.wav`:

```c
extern const uint8_t myfile_wav_start[] asm("_binary_myfile_wav_start");
extern const uint8_t myfile_wav_end[]   asm("_binary_myfile_wav_end");
```

## WAV Requirements

- Format: PCM (uncompressed)
- Bit depth: 8, 16, 24, or 32-bit
- Channels: mono or stereo
- Sample rate: any (auto-detected from WAV header)

## Project Structure

```
esp32p4-audio/
├── CMakeLists.txt              # Project-level CMake
├── main/
│   ├── CMakeLists.txt          # Component CMake (auto-embeds WAV files)
│   ├── main.c                  # I2S driver + WAV parser
│   └── audio/
│       └── beep.wav            # Sample 1kHz beep (44100Hz, 16-bit, mono)
├── sdkconfig.defaults          # Default Kconfig values
└── README.md
```
