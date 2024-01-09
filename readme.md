# ESP32 with PCM1808 ADC and PCM5102 DAC Sample

## Overview

This is an example project for grabbing data from PCM1808 ADC and sending it to PCM5102 DAC. Additionally the ESP32 DAC peripheral is used to generate a 440 Hz test tone on the DAC CH0 output pin (25).

## Pinout

```
#define I2S_STD_MCLK_IO1        GPIO_NUM_0    // I2S master clock io number
#define I2S_STD_BCLK_IO1        GPIO_NUM_2      // I2S bit clock io number
#define I2S_STD_WS_IO1          GPIO_NUM_15      // I2S word select io number
#define I2S_STD_DOUT_IO1        GPIO_NUM_4      // I2S data out io number
#define I2S_STD_DIN_IO1         GPIO_NUM_16      // I2S data in io number
```

| Function | ESP32 Pin |
| -------- | ---------:|
| SCK/MCLK | 0         |
| BCLK     | 2         |
| WS/LRCK  | 15        |
| ESP32 DOUT | 4       |
| ESP32 DIN | 16       |
| DAC OUT | 25       |