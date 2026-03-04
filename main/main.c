/**
 * ESP32-P4 Audio Player with MAX98357A
 *
 * Plays embedded WAV files via I2S using the new ESP-IDF 5.5.x driver API.
 * WAV files in main/audio/ are automatically embedded at compile time.
 *
 * I2S Pins (MAX98357A):
 *   LRC  (WS)   -> GPIO 15
 *   BCLK        -> GPIO 14
 *   DIN  (DOUT) -> GPIO 13
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "audio_player";

/* ── Volume Control ────────────────────────────────────────────────────── */
static volatile uint8_t volume_level = 80;  /* 0 (mute) to 100 (max) */

/**
 * Set playback volume. Range: 0 (mute) to 100 (full volume).
 * Can be called from any task at any time — takes effect on the next audio chunk.
 */
void audio_set_volume(uint8_t level)
{
    if (level > 100) level = 100;
    volume_level = level;
    ESP_LOGI(TAG, "Volume set to %d%%", level);
}

uint8_t audio_get_volume(void)
{
    return volume_level;
}

/* ── I2S Pin Configuration ─────────────────────────────────────────────── */
#define I2S_LRC_GPIO    GPIO_NUM_15   /* Word Select (LR Clock) */
#define I2S_BCLK_GPIO   GPIO_NUM_14   /* Bit Clock */
#define I2S_DIN_GPIO    GPIO_NUM_13   /* Data Out to MAX98357A */

/* ── Embedded WAV file (beep.wav from main/audio/) ─────────────────────── */
extern const uint8_t beep_wav_start[] asm("_binary_beep_wav_start");
extern const uint8_t beep_wav_end[]   asm("_binary_beep_wav_end");

/* ── WAV Header Parser ─────────────────────────────────────────────────── */
typedef struct {
    uint32_t sample_rate;
    uint16_t num_channels;
    uint16_t bits_per_sample;
    uint16_t block_align;
    const uint8_t *data;
    uint32_t data_size;
} wav_info_t;

/**
 * Parse a WAV file buffer and extract audio parameters + PCM data pointer.
 * Handles WAV files with extra chunks between fmt and data.
 */
static esp_err_t wav_parse(const uint8_t *buf, size_t buf_size, wav_info_t *info)
{
    if (buf_size < 44) {
        ESP_LOGE(TAG, "Buffer too small for WAV header");
        return ESP_ERR_INVALID_SIZE;
    }

    /* Verify RIFF/WAVE header */
    if (memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Not a valid WAV file");
        return ESP_ERR_INVALID_ARG;
    }

    /* Find "fmt " chunk */
    const uint8_t *ptr = buf + 12;
    const uint8_t *end = buf + buf_size;
    bool fmt_found = false;

    while (ptr + 8 <= end) {
        uint32_t chunk_size = *(uint32_t *)(ptr + 4);

        if (memcmp(ptr, "fmt ", 4) == 0) {
            if (ptr + 8 + chunk_size > end) {
                ESP_LOGE(TAG, "fmt chunk extends beyond buffer");
                return ESP_ERR_INVALID_SIZE;
            }

            uint16_t audio_format = *(uint16_t *)(ptr + 8);
            if (audio_format != 1) {
                ESP_LOGE(TAG, "Only PCM format supported (got %d)", audio_format);
                return ESP_ERR_NOT_SUPPORTED;
            }

            info->num_channels   = *(uint16_t *)(ptr + 10);
            info->sample_rate    = *(uint32_t *)(ptr + 12);
            info->block_align    = *(uint16_t *)(ptr + 20);
            info->bits_per_sample = *(uint16_t *)(ptr + 22);
            fmt_found = true;
        }

        if (memcmp(ptr, "data", 4) == 0) {
            if (!fmt_found) {
                ESP_LOGE(TAG, "data chunk found before fmt chunk");
                return ESP_ERR_INVALID_STATE;
            }
            info->data_size = chunk_size;
            info->data = ptr + 8;

            /* Clamp data_size if it extends beyond buffer */
            if (info->data + info->data_size > end) {
                info->data_size = end - info->data;
            }

            ESP_LOGI(TAG, "WAV: %lu Hz, %d-bit, %d ch, %lu bytes PCM",
                     (unsigned long)info->sample_rate,
                     info->bits_per_sample,
                     info->num_channels,
                     (unsigned long)info->data_size);
            return ESP_OK;
        }

        ptr += 8 + chunk_size;
        /* Chunks are word-aligned */
        if (chunk_size & 1) ptr++;
    }

    ESP_LOGE(TAG, "data chunk not found in WAV file");
    return ESP_ERR_NOT_FOUND;
}

/* ── I2S Setup & Playback ──────────────────────────────────────────────── */
static i2s_chan_handle_t tx_handle = NULL;

/**
 * Initialize I2S in standard (Philips) mode for MAX98357A.
 */
static esp_err_t i2s_init(const wav_info_t *wav)
{
    /* Determine I2S parameters from WAV info */
    i2s_data_bit_width_t bit_width;
    switch (wav->bits_per_sample) {
        case 8:  bit_width = I2S_DATA_BIT_WIDTH_8BIT;  break;
        case 16: bit_width = I2S_DATA_BIT_WIDTH_16BIT; break;
        case 24: bit_width = I2S_DATA_BIT_WIDTH_24BIT; break;
        case 32: bit_width = I2S_DATA_BIT_WIDTH_32BIT; break;
        default:
            ESP_LOGE(TAG, "Unsupported bit depth: %d", wav->bits_per_sample);
            return ESP_ERR_NOT_SUPPORTED;
    }

    i2s_slot_mode_t slot_mode = (wav->num_channels >= 2)
        ? I2S_SLOT_MODE_STEREO
        : I2S_SLOT_MODE_MONO;

    /* Step 1: Allocate TX channel */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear_after_cb = true;   /* Zero DMA buffers after TX done — prevents stale data noise */
    chan_cfg.auto_clear_before_cb = true;  /* Zero DMA buffers before filling — clean start */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    /* Step 2: Configure standard mode (Philips) */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(wav->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width, slot_mode),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws   = I2S_LRC_GPIO,
            .dout = I2S_DIN_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    /* For 24-bit audio, use MCLK multiple of 384 for clean division */
    if (wav->bits_per_sample == 24) {
        std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
    }

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));

    /* Step 3: Enable the channel */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGI(TAG, "I2S initialized: %lu Hz, %d-bit, %s",
             (unsigned long)wav->sample_rate,
             wav->bits_per_sample,
             (slot_mode == I2S_SLOT_MODE_STEREO) ? "stereo" : "mono");

    return ESP_OK;
}

/**
 * Apply volume scaling to 16-bit PCM samples in a buffer.
 * Uses fixed-point multiplication (vol 0-100 → scale 0-256) for efficiency.
 */
static void apply_volume_16bit(const uint8_t *src, uint8_t *dst, size_t bytes, uint8_t vol)
{
    const int16_t *in  = (const int16_t *)src;
    int16_t       *out = (int16_t *)dst;
    size_t num_samples = bytes / sizeof(int16_t);
    uint32_t scale = (uint32_t)vol * 256 / 100;  /* 0-256 fixed point */

    for (size_t i = 0; i < num_samples; i++) {
        out[i] = (int16_t)(((int32_t)in[i] * scale) >> 8);
    }
}

/**
 * Play the PCM data from a parsed WAV file through I2S with volume control.
 */
static esp_err_t play_wav(const wav_info_t *wav)
{
    const size_t chunk_size = 1024;  /* Write in 1KB chunks */
    uint8_t scaled_buf[1024];        /* Temp buffer for volume-scaled data */
    size_t remaining = wav->data_size;
    const uint8_t *ptr = wav->data;

    while (remaining > 0) {
        size_t to_write = (remaining < chunk_size) ? remaining : chunk_size;
        size_t bytes_written = 0;
        uint8_t vol = volume_level;

        /* Apply volume scaling (currently supports 16-bit; other depths pass through) */
        const uint8_t *write_ptr;
        if (wav->bits_per_sample == 16 && vol < 100) {
            apply_volume_16bit(ptr, scaled_buf, to_write, vol);
            write_ptr = scaled_buf;
        } else if (vol == 0) {
            memset(scaled_buf, 0, to_write);
            write_ptr = scaled_buf;
        } else {
            write_ptr = ptr;  /* Full volume or non-16-bit: pass through unchanged */
        }

        esp_err_t ret = i2s_channel_write(tx_handle, write_ptr, to_write, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write error: %s", esp_err_to_name(ret));
            return ret;
        }

        ptr += bytes_written;
        remaining -= bytes_written;
    }

    return ESP_OK;
}

/**
 * Flush DMA buffers with silence, then disable the I2S channel to stop all output.
 * This prevents stale audio data from replaying as noise between playbacks.
 */
static void stop_playback(void)
{
    /* Write enough silence to flush all DMA descriptors (default: 6 x 240 frames) */
    uint8_t silence[1024];
    memset(silence, 0, sizeof(silence));
    size_t written;
    for (int i = 0; i < 8; i++) {
        i2s_channel_write(tx_handle, silence, sizeof(silence), &written, portMAX_DELAY);
    }

    /* Disable the channel — stops the I2S clock and all output */
    i2s_channel_disable(tx_handle);
}

/* ── Main ──────────────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 Audio Player with MAX98357A");
    ESP_LOGI(TAG, "Pins: BCLK=GPIO%d, LRC=GPIO%d, DIN=GPIO%d",
             I2S_BCLK_GPIO, I2S_LRC_GPIO, I2S_DIN_GPIO);

    /* Parse the embedded WAV file */
    wav_info_t wav = {0};
    size_t wav_size = beep_wav_end - beep_wav_start;
    ESP_LOGI(TAG, "Embedded WAV size: %u bytes", (unsigned)wav_size);

    esp_err_t ret = wav_parse(beep_wav_start, wav_size, &wav);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse WAV file");
        return;
    }

    /* Initialize I2S based on WAV parameters */
    ret = i2s_init(&wav);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S");
        return;
    }

    /* Demo: play the beep at decreasing volume levels */
    uint8_t demo_volumes[] = {100, 70, 40, 20, 10};
    for (int i = 0; i < sizeof(demo_volumes); i++) {
        audio_set_volume(demo_volumes[i]);
        ESP_LOGI(TAG, "Playing beep at %d%% volume...", demo_volumes[i]);

        if (i > 0) {
            i2s_channel_enable(tx_handle);
        }
        play_wav(&wav);
        stop_playback();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Then loop at default volume */
    audio_set_volume(80);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Playing beep at %d%% volume...", audio_get_volume());
        i2s_channel_enable(tx_handle);
        play_wav(&wav);
        stop_playback();
    }
}
