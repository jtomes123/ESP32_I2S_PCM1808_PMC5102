#include <Arduino.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/dac_cosine.h"

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

#define AUDIO_BUFF_SIZE       4096*2            // Audio buffer size

/* For ESP32-S2
#define I2S_STD_MCLK_IO1        GPIO_NUM_7      // I2S master clock io number
#define I2S_STD_BCLK_IO1        GPIO_NUM_2      // I2S bit clock io number
#define I2S_STD_WS_IO1          GPIO_NUM_3      // I2S word select io number
#define I2S_STD_DOUT_IO1        GPIO_NUM_4      // I2S data out io number
#define I2S_STD_DIN_IO1         GPIO_NUM_5      // I2S data in io number
*/

/* For ESP32 */
#define I2S_STD_MCLK_IO1        GPIO_NUM_0    // I2S master clock io number
#define I2S_STD_BCLK_IO1        GPIO_NUM_2      // I2S bit clock io number
#define I2S_STD_WS_IO1          GPIO_NUM_15      // I2S word select io number
#define I2S_STD_DOUT_IO1        GPIO_NUM_4      // I2S data out io number
#define I2S_STD_DIN_IO1         GPIO_NUM_16      // I2S data in io number

void i2s_setup();
void dac_setup();
static void i2s_example_read_task(void* args);
static void i2s_example_write_task(void* args);
static void i2s_example_dsp_task(void* args);

// Set up the DAC peripheral to produce 440 Hz tone on GPIO25
void dac_setup()
{
  dac_cosine_handle_t chan0_handle;
  dac_cosine_config_t cos0_cfg = {
      .chan_id = DAC_CHAN_0,
      .freq_hz = 440,
      .clk_src = DAC_COSINE_CLK_SRC_DEFAULT,
      .atten = DAC_COSINE_ATTEN_DEFAULT,
      .offset = 0,
  };
     
  cos0_cfg.phase = DAC_COSINE_PHASE_0;
  cos0_cfg.flags.force_set_freq = false;
  
  ESP_ERROR_CHECK(dac_cosine_new_channel(&cos0_cfg, &chan0_handle));
  ESP_ERROR_CHECK(dac_cosine_start(chan0_handle));
}

void i2s_setup() {
  /* Allocate a pair of I2S channel */
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  /* Allocate for TX and RX channel at the same time, then they will work in full-duplex mode */
  Serial.printf("Init I2S Channel: %x\n", i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
  /* Set the configurations for BOTH TWO channels, since TX and RX channel have to be same in full-duplex mode */
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_STD_MCLK_IO1,
          .bclk = I2S_STD_BCLK_IO1,
          .ws = I2S_STD_WS_IO1,
          .dout = I2S_STD_DOUT_IO1,
          .din = I2S_STD_DIN_IO1,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          },
      },
  };
  std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  Serial.printf("Init I2S TX: %x\n", i2s_channel_init_std_mode(tx_handle, &std_cfg));
  Serial.printf("Init I2S RX: %x\n", i2s_channel_init_std_mode(rx_handle, &std_cfg));

  Serial.printf("Enable I2S TX: %x\n", i2s_channel_enable(tx_handle));
  Serial.printf("Enable I2S RX: %x\n", i2s_channel_enable(rx_handle));
}

void setup() {

  // Set up Serial Monitor
  Serial.begin(115200);
  Serial.println(" ");

  delay(1000);

  // Set up I2S
  i2s_setup();

  //Set up dac to generate 440Hz test tone
  dac_setup();

  delay(500);

  // Loopback/DSP task
  xTaskCreate(i2s_example_dsp_task, "i2s_example_dsp_task", 4096, NULL, 5, NULL);

  // Debug tasks
  //xTaskCreate(i2s_example_read_task, "i2s_example_read_task", 4096, NULL, 5, NULL);
  //xTaskCreate(i2s_example_write_task, "i2s_example_write_task", 4096, NULL, 5, NULL);
}

void loop() {
  delay(1000);
}

// Modified read example
static void i2s_example_read_task(void* args)
{
  uint8_t* r_buf = (uint8_t*)calloc(1, AUDIO_BUFF_SIZE);
  assert(r_buf); // Check if r_buf allocation success
  size_t r_bytes = 0;

  /* Enable the RX channel */
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

  /* Print might make this loop way too slow, consider limiting amount of info printed and lowering the sample rate */
  while (1) {
    /* Read i2s data */
    if (i2s_channel_read(rx_handle, r_buf, AUDIO_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) 
    {
      Serial.printf("%" PRId32  "\n", (r_buf[0] << 24) | (r_buf[1] << 16) | (r_buf[2] << 8) | (r_buf[3]));
    }
    else {
      Serial.printf("Read Task: i2s read failed\n");
    }
    vTaskDelay(pdMS_TO_TICKS(16));
  }
  free(r_buf);
  vTaskDelete(NULL);
}

static void i2s_example_write_task(void* args)
{
  uint8_t* w_buf = (uint8_t*)calloc(1, AUDIO_BUFF_SIZE);
  assert(w_buf); // Check if w_buf allocation success

  /* Assign w_buf */
  for (int i = 0; i < AUDIO_BUFF_SIZE; i += 8) {
    w_buf[i] = 0x12;
    w_buf[i + 1] = 0x34;
    w_buf[i + 2] = 0x56;
    w_buf[i + 3] = 0x78;
    w_buf[i + 4] = 0x9A;
    w_buf[i + 5] = 0xBC;
    w_buf[i + 6] = 0xDE;
    w_buf[i + 7] = 0xF0;
  }

  size_t w_bytes = AUDIO_BUFF_SIZE;

  Serial.println("Preloading audio.");
  /* Enable the TX channel */
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

  // DO NOT PRELOAD DATA - IT FAILS/DEADLOCKS

  Serial.println("Getting ready for write.");

  while (1) {
    /* Write i2s data */
    esp_err_t err = i2s_channel_write(tx_handle, w_buf, AUDIO_BUFF_SIZE, &w_bytes, 1000);
    if (err == ESP_OK) {
      Serial.printf("Write Task: i2s write %d bytes\n", w_bytes);
    }
    else {
      Serial.printf("Write Task: i2s write failed with code: %d\n", err);
    }
    Serial.println("Write loop done.");
    vTaskDelay(pdMS_TO_TICKS(16));
  }
  free(w_buf);
  vTaskDelete(NULL);
}

static void i2s_example_dsp_task(void* args)
{
  size_t w_bytes = AUDIO_BUFF_SIZE;

  /* Enable the TX channel */
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

  uint8_t* r_buf = (uint8_t*)calloc(1, AUDIO_BUFF_SIZE);
  assert(r_buf); // Check if r_buf allocation success
  size_t r_bytes = 0;

  /* Enable the RX channel */
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

  Serial.println("Ready for write.");

  while (1) {
    /* Read i2s data */
    auto err_r = i2s_channel_read(rx_handle, r_buf, AUDIO_BUFF_SIZE, &r_bytes, 1000);
    if (err_r != ESP_OK) {
      Serial.printf("DSP Task: i2s read failed with code: %d\n", err_r);
    }

    // TODO: Add audio processing here

    /* Write i2s data */
    auto err_w = i2s_channel_write(tx_handle, r_buf, r_bytes, &w_bytes, 1000);
    if (err_w != ESP_OK) {
      Serial.printf("DSP Task: i2s write failed with code: %d\n", err_w);
    }

    vTaskDelay(pdMS_TO_TICKS(16));
  }

  free(r_buf);
  vTaskDelete(NULL);
}