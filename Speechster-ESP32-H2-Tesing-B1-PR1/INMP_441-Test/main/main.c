#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"

#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     16
#define CHANNELS            1       // Mono
#define RECORD_DURATION_SEC 5
#define BUFFER_SIZE         (SAMPLE_RATE * (BITS_PER_SAMPLE / 8) * CHANNELS * RECORD_DURATION_SEC)

#define I2S_NUM             I2S_NUM_AUTO
#define UART_NUM            UART_NUM_0

static const char *TAG = "INMP441";

i2s_chan_handle_t rx_handle = NULL;

/**
 * @brief Generates a simple WAV header
 */
void generate_wav_header(uint8_t *wav_header, uint32_t data_length, uint32_t sample_rate, uint16_t bits_per_sample, uint16_t channels)
{
    uint32_t byte_rate = sample_rate * channels * bits_per_sample / 8;
    uint16_t block_align = channels * bits_per_sample / 8;

    // RIFF chunk
    memcpy(wav_header, "RIFF", 4);
    *(uint32_t *)(wav_header + 4) = 36 + data_length;         // ChunkSize
    memcpy(wav_header + 8, "WAVE", 4);

    // fmt subchunk
    memcpy(wav_header + 12, "fmt ", 4);
    *(uint32_t *)(wav_header + 16) = 16;                      // Subchunk1Size for PCM
    *(uint16_t *)(wav_header + 20) = 1;                       // AudioFormat = PCM
    *(uint16_t *)(wav_header + 22) = channels;
    *(uint32_t *)(wav_header + 24) = sample_rate;
    *(uint32_t *)(wav_header + 28) = byte_rate;
    *(uint16_t *)(wav_header + 32) = block_align;
    *(uint16_t *)(wav_header + 34) = bits_per_sample;

    // data subchunk
    memcpy(wav_header + 36, "data", 4);
    *(uint32_t *)(wav_header + 40) = data_length;
}

void record_and_stream_audio()
{
    uint8_t *buffer = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DEFAULT);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        return;
    }

    size_t bytes_read = 0;
    size_t total_read = 0;
    esp_err_t ret;

    ESP_LOGI(TAG, "Recording audio for %d seconds...", RECORD_DURATION_SEC);

    // Read samples from I2S into buffer
    while (total_read < BUFFER_SIZE) {
        ret = i2s_channel_read(rx_handle, buffer + total_read, BUFFER_SIZE - total_read, &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S read failed: %s", esp_err_to_name(ret));
            break;
        }
        total_read += bytes_read;
    }

    ESP_LOGI(TAG, "Recording complete. Sending WAV data...");

    // Generate WAV header
    uint8_t wav_header[44];
    generate_wav_header(wav_header, BUFFER_SIZE, SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS);

    // Send data to UART
    const char *start_tag = "StartAudioWAV\n";
    const char *stop_tag = "StopAudioWAV\n";
    uart_write_bytes(UART_NUM, start_tag, strlen(start_tag));
    uart_write_bytes(UART_NUM, (const char *)wav_header, sizeof(wav_header));
    uart_write_bytes(UART_NUM, (const char *)buffer, BUFFER_SIZE);
    uart_write_bytes(UART_NUM, stop_tag, strlen(stop_tag));

    ESP_LOGI(TAG, "WAV transmission complete.");

    free(buffer);
}

void app_main(void)
{
    esp_err_t ret;

    // Configure UART (optional if UART0 already in use)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, 2048, 0, 0, NULL, 0);

    // Channel config
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = false,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
        .allow_pd = false
    };

    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    ESP_ERROR_CHECK(ret);

    // Clock config
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);

    // Slot config (16-bit, mono)
    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT,
        I2S_SLOT_MODE_MONO
    );

    // GPIO config for INMP441
    i2s_std_gpio_config_t gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = GPIO_NUM_10,
        .ws   = GPIO_NUM_11,
        .dout = I2S_GPIO_UNUSED,
        .din  = GPIO_NUM_12
    };

    i2s_std_config_t std_cfg = {
        .clk_cfg = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = gpio_cfg,
    };

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    ESP_ERROR_CHECK(ret);
    ret = i2s_channel_enable(rx_handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "I2S RX channel started. Ready to record.");

    // Loop to record and stream audio every 5 seconds
    while (1) {
        record_and_stream_audio();
        vTaskDelay(pdMS_TO_TICKS(6000));  // Add 1 sec pause between recordings
    }
}
