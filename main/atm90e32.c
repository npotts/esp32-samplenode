/* Dual use Weather and Splot Phase Energy Meter

Intended to be used with CircuitSetup.us Split Phase Energy Meter and some extra tacked on sensors.

The MIT License (MIT)
Copyright (c) 2019 npotts
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include <math.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/spi_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "atm90e32.h"
#include "mqtt.client.h"

static const char *TAG = "atm90e32";
spi_device_handle_t spi_dev; 

#define SPI_MASTER_FREQ_200K SPI_MASTER_FREQ_40M / 200

/*Writes the 2 byte ```value``` to the passed ```address```*/
esp_err_t spi_write_register(uint16_t address, uint16_t value) {
    address = 0x7fff & address; //clear top bit
    struct spi_transaction_t t = { .user = NULL, .cmd = address, .flags = SPI_TRANS_USE_TXDATA, .length = 16, .rxlength = 0, .rx_buffer = NULL, .tx_buffer = NULL };
    t.tx_data[0] = value >> 8; t.tx_data[1] = value & 0xFF;
    esp_err_t err = spi_device_polling_transmit(spi_dev, &t);
    // ESP_LOGW(TAG, "Write [%d] DATA = %X-%X-%X-%X ERR=%d", address, t.tx_data[0], t.tx_data[1], t.tx_data[2], t.tx_data[3], err);
    return err;
}

/*attempt to read from ```address``` into ```data```*/
esp_err_t spi_read_register(uint16_t address, uint16_t *data) {
    address = 1<<15 | address; //set top bit
    struct spi_transaction_t t = { .user = NULL, .cmd = address, .flags =  SPI_TRANS_USE_RXDATA, .rxlength = 16, .tx_buffer = NULL, .rx_buffer = NULL, };
    esp_err_t err = spi_device_polling_transmit(spi_dev, &t);
    uint16_t d = (t.rx_data[0] << 8) |  t.rx_data[1];
    memcpy(data, &d, 2);
    // ESP_LOGI(TAG, "Read [%x] DATA = %X-%X-%X-%X %d ERR=%d", address, t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3], d, err);
    return err;   
}

/*bootstrap the configuration of the ATM90E32*/
void configure_atm90e32(void) {
    #if CONFIG_ATM90E32_LINE_FREQ==4485 || CONFIG_ATM90E32_LINE_FREQ==523
    uint16_t sagV = 90;
    uint16_t FreqHiThresh = 61 * 100;
    uint16_t FreqLoThresh = 59 * 100;
    #else
    uint16_t sagV = 190;
    uint16_t FreqHiThresh = 51 * 100;
    uint16_t FreqLoThresh = 49 * 100;
    #endif
    uint16_t vSagTh = (sagV * 100 * sqrt(2)) / (2 * CONFIG_ATM90E32_TRANSFORMER_VOLTAGE_GAIN / 32768);

    //Initialize registers
    spi_write_register(ATM90E32_SoftReset, 0x789A);     // 70 Perform soft reset
    spi_write_register(ATM90E32_CfgRegAccEn, 0x55AA);   // 7F enable register config access
    spi_write_register(ATM90E32_MeterEn, 0x0001);       // 00 Enable Metering

    spi_write_register(ATM90E32_SagPeakDetCfg, 0x143F);   // 05 Sag and Voltage peak detect period set to 20ms
    spi_write_register(ATM90E32_SagTh,         vSagTh);   // 08 Voltage sag threshold
    spi_write_register(ATM90E32_FreqHiTh,  FreqHiThresh);  // 0D High frequency threshold
    spi_write_register(ATM90E32_FreqLoTh,  FreqLoThresh);  // 0C Lo frequency threshold
    spi_write_register(ATM90E32_EMMIntEn0, 0xB76F);     // 75 Enable interrupts
    spi_write_register(ATM90E32_EMMIntEn1, 0xDDFD);     // 76 Enable interrupts
    spi_write_register(ATM90E32_EMMIntState0, 0x0001);  // 73 Clear interrupt flags
    spi_write_register(ATM90E32_EMMIntState1, 0x0001);  // 74 Clear interrupt flags
    spi_write_register(ATM90E32_ZXConfig, 0xD654);      // 07 ZX2, ZX1, ZX0 pin config - set to current channels, all polarity

    //Set metering config values (CONFIG)
    spi_write_register(ATM90E32_PLconstH, 0x0861);    // 31 PL Constant MSB (default) - Meter Constant = 3200 - PL Constant = 140625000
    spi_write_register(ATM90E32_PLconstL, 0xC468);    // 32 PL Constant LSB (default) - this is 4C68 in the application note, which is incorrect
    spi_write_register(ATM90E32_MMode0, CONFIG_ATM90E32_LINE_FREQ);   // 33 Mode Config (frequency set in main program)
    spi_write_register(ATM90E32_MMode1, CONFIG_ATM90E32_PGAGAIN);    // 34 PGA Gain Configuration for Current Channels - 0x002A (x4) // 0x0015 (x2) // 0x0000 (1x)
    spi_write_register(ATM90E32_PStartTh, 0x1D4C);    // 35 All phase Active Startup Power Threshold - 50% of startup current = 0.02A/0.00032 = 7500
    spi_write_register(ATM90E32_QStartTh, 0x1D4C);    // 36 All phase Reactive Startup Power Threshold
    spi_write_register(ATM90E32_SStartTh, 0x1D4C);    // 37 All phase Apparent Startup Power Threshold
    spi_write_register(ATM90E32_PPhaseTh, 0x02EE);    // 38 Each phase Active Phase Threshold = 10% of startup current = 0.002A/0.00032 = 750
    spi_write_register(ATM90E32_QPhaseTh, 0x02EE);    // 39 Each phase Reactive Phase Threshold
    spi_write_register(ATM90E32_SPhaseTh, 0x02EE);    // 3A Each phase Apparent Phase Threshold

    //Set metering calibration values (CALIBRATION)
    spi_write_register(ATM90E32_PQGainA,  0x0000);     // 47 Line calibration gain
    spi_write_register(ATM90E32_PhiA,     0x0000);     // 48 Line calibration angle
    spi_write_register(ATM90E32_PQGainB,  0x0000);     // 49 Line calibration gain
    spi_write_register(ATM90E32_PhiB,     0x0000);     // 4A Line calibration angle
    spi_write_register(ATM90E32_PQGainC,  0x0000);     // 4B Line calibration gain
    spi_write_register(ATM90E32_PhiC,     0x0000);     // 4C Line calibration angle
    spi_write_register(ATM90E32_PoffsetA, 0x0000);     // 41 A line active power offset FFDC
    spi_write_register(ATM90E32_QoffsetA, 0x0000);     // 42 A line reactive power offset
    spi_write_register(ATM90E32_PoffsetB, 0x0000);     // 43 B line active power offset
    spi_write_register(ATM90E32_QoffsetB, 0x0000);     // 44 B line reactive power offset
    spi_write_register(ATM90E32_PoffsetC, 0x0000);     // 45 C line active power offset
    spi_write_register(ATM90E32_QoffsetC, 0x0000);     // 46 C line reactive power offset

    //Set metering calibration values (HARMONIC)
    spi_write_register(ATM90E32_POffsetAF, 0x0000);   // 51 A Fund. active power offset
    spi_write_register(ATM90E32_POffsetBF, 0x0000);   // 52 B Fund. active power offset
    spi_write_register(ATM90E32_POffsetCF, 0x0000);   // 53 C Fund. active power offset
    spi_write_register(ATM90E32_PGainAF,   0x0000);   // 54 A Fund. active power gain
    spi_write_register(ATM90E32_PGainBF,   0x0000);   // 55 B Fund. active power gain
    spi_write_register(ATM90E32_PGainCF,   0x0000);   // 56 C Fund. active power gain

    //Set measurement calibration values (ADJUST)
    spi_write_register(ATM90E32_UgainA, CONFIG_ATM90E32_TRANSFORMER_VOLTAGE_GAIN);      // 61 A Voltage rms gain
    spi_write_register(ATM90E32_IgainA, CONFIG_ATM90E32_CURRENT_GAIN);     // 62 A line current gain
    spi_write_register(ATM90E32_UoffsetA, 0x0000);    // 63 A Voltage offset - 61A8
    spi_write_register(ATM90E32_IoffsetA, 0x0000);    // 64 A line current offset - FE60
    spi_write_register(ATM90E32_UgainB, CONFIG_ATM90E32_TRANSFORMER_VOLTAGE_GAIN);      // 65 B Voltage rms gain
    spi_write_register(ATM90E32_IgainB, 0);     // 66 B line current gain
    spi_write_register(ATM90E32_UoffsetB, 0x0000);    // 67 B Voltage offset - 1D4C
    spi_write_register(ATM90E32_IoffsetB, 0x0000);    // 68 B line current offset - FE60
    spi_write_register(ATM90E32_UgainC, CONFIG_ATM90E32_TRANSFORMER_VOLTAGE_GAIN);      // 69 C Voltage rms gain
    spi_write_register(ATM90E32_IgainC, CONFIG_ATM90E32_CURRENT_GAIN);     // 6A C line current gain
    spi_write_register(ATM90E32_UoffsetC, 0x0000);    // 6B C Voltage offset - 1D4C
    spi_write_register(ATM90E32_IoffsetC, 0x0000);    // 6C C line current offset

    spi_write_register(ATM90E32_CfgRegAccEn, 0x0000); // 7F end configuration
}

sample_t readDouble(uint16_t addr, double multiplier) {
    uint16_t val;
    esp_err_t err = spi_read_register(addr, &val);
    return update_value_f( (double)val * multiplier, err);
}
sample_t read32Double(uint16_t addr_a, uint16_t addr_b, double multiplier) {
    uint16_t val;
    esp_err_t err = spi_read_register(addr_a, &val);
    uint32_t a = val << 16;
    err  |= spi_read_register(addr_b, &val);
    a = a | val;
    return update_value_f( (double) a * multiplier, err);
}

pwd_data_t pwr_data;
/*sample_data fetches records from the SPI bus and spews them to MQTT*/
void sample_data(void) {
    pwr_data.v1 = readDouble(ATM90E32_UrmsA, 0.01);
    pwr_data.v2 = readDouble(ATM90E32_UrmsC, 0.01);
    pwr_data.i1 = readDouble(ATM90E32_IrmsA, 0.01);
    pwr_data.i2 = readDouble(ATM90E32_IrmsC, 0.01);
    pwr_data.ap = read32Double(ATM90E32_SmeanT, ATM90E32_SAmeanTLSB, 0.00032);
    pwr_data.pf = readDouble(ATM90E32_PFmeanT, 0.001);
    pwr_data.t = readDouble(ATM90E32_Temp, 1.0);
    pwr_data.f = readDouble(ATM90E32_Freq, 0.01);
    pwr_data.w = read32Double(ATM90E32_PmeanTF, ATM90E32_PmeanTFLSB, 1.0);
    pwr_data.I = update_value_f(pwr_data.i1.sample.i + pwr_data.i2.sample.i, pwr_data.i1.error | pwr_data.i2.error );

    broadcast_sample(pwr_data.v1, CONFIG_MQTT_TOPIC_POWER"/v1");
    broadcast_sample(pwr_data.v2, CONFIG_MQTT_TOPIC_POWER "/v2");
    broadcast_sample(pwr_data.i1, CONFIG_MQTT_TOPIC_POWER "/i1");
    broadcast_sample(pwr_data.i2, CONFIG_MQTT_TOPIC_POWER "/i2");
    broadcast_sample(pwr_data.I, CONFIG_MQTT_TOPIC_POWER "/I");
    broadcast_sample(pwr_data.ap, CONFIG_MQTT_TOPIC_POWER "/ap");
    broadcast_sample(pwr_data.pf, CONFIG_MQTT_TOPIC_POWER "/pf");
    broadcast_sample(pwr_data.t, CONFIG_MQTT_TOPIC_POWER "/t");
    broadcast_sample(pwr_data.f, CONFIG_MQTT_TOPIC_POWER "/f");
    broadcast_sample(pwr_data.w, CONFIG_MQTT_TOPIC_POWER "/w");
    ESP_LOGI(TAG, "published power data");
}

void atm90e32_monitor(void *parameter) {
  ESP_LOGI(TAG, "ATM90E32 reading routine");
  configure_atm90e32();
  uint16_t buf;
  char errmsg[256];
  for (;;) {
    spi_read_register(ATM90E32_MeterEn, &buf);
    if (buf != 1) {
        configure_atm90e32(); //TODO: ALert on misconfigured power measurement
        int n = snprintf(errmsg, sizeof(errmsg), "[%d] ATM90E32 seems broken or not responding",  esp_log_timestamp());
        if (mqtt_publish_msg(CONFIG_MQTT_TOPIC_HEARTBEAT "/pwr", errmsg, n) == -1) {
            ESP_LOGE(TAG, "Unable to publish heartbeat");
        }
    }
    sample_data();
    vTaskDelay(pdMS_TO_TICKS(CONFIG_PWR_POLL_PERIOD));
  }
}

void atm_init(void) {
    /*Init atm90e32 device  generally this is connected to V-SPI lines */
    const spi_bus_config_t bus_cfg = {
        .mosi_io_num = VSPI_IOMUX_PIN_NUM_MOSI, //23
        .miso_io_num = VSPI_IOMUX_PIN_NUM_MISO, //19
        .sclk_io_num = VSPI_IOMUX_PIN_NUM_CLK,  //18
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    esp_err_t err = spi_bus_initialize(VSPI_HOST, &bus_cfg, 0);
    switch (err) {
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(TAG, "Unable to start SPI bus");
            break;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGE(TAG, "SPI host already is in use");
            break;
        case ESP_ERR_NO_MEM:
            ESP_LOGE(TAG, "SPI if out of memory");
            break;
        case ESP_OK:
            ESP_LOGI(TAG, "SPI started");
            break;
        default:
            ESP_LOGE(TAG, "Oh snap!  Something else went wrong: %d", err);
            break;
    }
    
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 16,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode=0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans=1,
        .cs_ena_posttrans=1,
        .clock_speed_hz=SPI_MASTER_FREQ_200K,
        .input_delay_ns = 0,
        .spics_io_num = VSPI_IOMUX_PIN_NUM_CS,
        .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX,
        .queue_size = 16,
    };

    err =  spi_bus_add_device(VSPI_HOST, &dev_cfg, &spi_dev);
    switch (err) {
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(TAG, "if parameter is invalid");
            break;
        case ESP_ERR_NOT_FOUND:
            ESP_LOGE(TAG, "host doesn't have any free CS slots");
            break;
        case ESP_ERR_NO_MEM:
            ESP_LOGE(TAG, "SPI out of memory");
            break;
        case ESP_OK:
            ESP_LOGI(TAG, "SPI started");
            break;
        default:
            ESP_LOGE(TAG, "Oh snap!  Something else went wrong: %d", err);
            break;
    }
    
    xTaskCreate(atm90e32_monitor,"atm90e32", 4096, 0 ,1, 0);
}