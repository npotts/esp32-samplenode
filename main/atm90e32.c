/* Dual use Weather and Splot Phase Energy Meter

Intended to be used with CircuitSetup.us Split Phase Energy Meter and some extra tacked on sensors.

The MIT License (MIT)
Copyright (c) 2019 npotts
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/spi_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "atm90e32.h"

static const char *TAG = "atm90e32";
spi_device_handle_t spi_dev; 

#define SPI_MASTER_FREQ_200K SPI_MASTER_FREQ_40M / 200

esp_err_t atm_write_register(uint16_t address, uint32_t value) {
    /*Writes the 2 byte ```value``` to the passed register*/
    address = 0x7fff & address; //clear top bit
    // address = (address >> 8) | (address << 8);
    
    struct spi_transaction_t trans = {
        .user = NULL,
        .flags  =  SPI_TRANS_SET_CD|  SPI_TRANS_USE_TXDATA,
        .addr = address, //set R/~W bit
        .length = 2, .rxlength = 2,
        .rx_buffer = NULL,
        .tx_buffer = NULL,
    };
    memcpy(trans.tx_data, (void *) &value, 4);
    ESP_LOGI(TAG, "Write DATA = %X-%X-%X-%X", trans.tx_data[0], trans.tx_data[1], trans.tx_data[2], trans.tx_data[3]);
    esp_err_t err = spi_device_polling_transmit(spi_dev, &trans);
    
    return err;

}

esp_err_t atm_read_register(uint16_t address, uint32_t *data) {
    /*attempt to read up to ```len``` bytes at ```address``` into data len will be forced to 4 if > 4 */
    address = 1<<15 | address; //set top bit
    address = (address >> 8) | (address << 8);
    ESP_LOGI(TAG, "Read using ADDR = %X", address);
    struct spi_transaction_t trans = {
        .user = NULL,
        .flags  = SPI_TRANS_SET_CD | SPI_TRANS_USE_RXDATA /* | SPI_TRANS_USE_TXDATA*/,
        .addr = address,
        .length = 4,
        .rxlength = 4,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    esp_err_t err = spi_device_polling_transmit(spi_dev, &trans);
    ESP_LOGI(TAG, "Read DATA = %X-%X-%X-%X ERR=%d", trans.rx_data[0], trans.rx_data[1], trans.rx_data[2], trans.rx_data[3], err);
    memcpy(data, trans.rx_data, 2);
    return err;   
}

void readme(void *parameter) {
  ESP_LOGI(TAG, "ATM90E32 reading routine");
  uint16_t buf;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_err_t err = atm_write_register(ATM90E32_MeterEn, 0x12345678);
    // ESP_LOGW(TAG, "Write: err=%d %s", err, esp_err_to_name(err) );
    vTaskDelay(pdMS_TO_TICKS(1000));
    err = atm_read_register(ATM90E32_MeterEn, &buf);
    // ESP_LOGW(TAG, "Read: %X / err=%d %s", buf, err, esp_err_to_name(err) );
  }
}

void atm_init(void) {
    /*Init atm90e32 device  generally this is connected to V-SPI lines */
    const spi_bus_config_t bus_cfg = {
        .mosi_io_num = VSPI_IOMUX_PIN_NUM_MOSI, //23
        .miso_io_num = VSPI_IOMUX_PIN_NUM_MISO, //19
        .sclk_io_num = VSPI_IOMUX_PIN_NUM_CLK, //18
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,
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
        .command_bits = 0, //no command bits
        .address_bits = 16, //16 bit addresses, upper bit is RW bit
        .dummy_bits = 1,   //WANT 4uS < Amount of dummy bits to insert between address and data phase
        .mode=3,                   ///< SPI mode (0-3)
        .duty_cycle_pos = 0,         ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
        .cs_ena_pretrans=1,        ///want: 10us < Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
        .cs_ena_posttrans=1,      /// want: 10us? < Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
        .clock_speed_hz=SPI_MASTER_FREQ_200K,             ///< Clock speed, divisors of 80MHz, in Hz. See ``SPI_MASTER_FREQ_*``.
        .input_delay_ns = 0,       /**< Maximum data valid time of slave. The time required between SCLK and MISO
        valid, including the possible clock delay from slave to master. The driver uses this value to give an extra
        delay before the MISO is ready on the line. Leave at 0 unless you know you need a delay. For better timing
        performance at high frequency (over 8MHz), it's suggest to have the right value.
        */
        .spics_io_num = VSPI_IOMUX_PIN_NUM_CS, ///< CS GPIO pin for this device, or -1 if not used
        .flags = SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_HALFDUPLEX,  ///< Bitwise OR of SPI_DEVICE_* flags
        .queue_size = 1,                 ///< Transaction queue size. This sets how many transactions can be 'in the air' (queued using spi_device_queue_trans but not yet finished using spi_device_get_trans_result) at the same time
    //     transaction_cb_t pre_cb;   /**< Callback to be called before a transmission is started.
    //                              *
    //                              *  This callback is called within interrupt
    //                              *  context should be in IRAM for best
    //                              *  performance, see "Transferring Speed"
    //                              *  section in the SPI Master documentation for
    //                              *  full details. If not, the callback may crash
    //                              *  during flash operation when the driver is
    //                              *  initialized with ESP_INTR_FLAG_IRAM.
    //                              */
    // transaction_cb_t post_cb;  /**< Callback to be called after a transmission has completed.
    //                              *
    //                              *  This callback is called within interrupt
    //                              *  context should be in IRAM for best
    //                              *  performance, see "Transferring Speed"
    //                              *  section in the SPI Master documentation for
    //                              *  full details. If not, the callback may crash
    //                              *  during flash operation when the driver is
    //                              *  initialized with ESP_INTR_FLAG_IRAM.
    //                              */
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
    
    xTaskCreate(readme,"atm",2048, 0 ,1, 0);
}