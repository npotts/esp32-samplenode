#include "shared-data.h"
#include "wxsensors.h"
#include "mqtt.client.h"

static const char *TAG = "wxsensors";

static gpio_num_t i2c_gpio_sda = 21;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

struct wx_data_t weather_data;

esp_err_t init_i2c_bus(const i2c_config_t *cfg) {
    esp_err_t err = i2c_driver_install(i2c_port, I2C_MODE_MASTER, 8, 8, ESP_INTR_FLAG_IRAM );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to initialize i2c_driver: %d: %s", err, esp_err_to_name(err) );
        return err;
    }

    err = i2c_param_config(i2c_port, cfg );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to i2c_param_config() i2c_driver: %d: %s", err, esp_err_to_name(err) );
    }
    return err;
}

esp_err_t i2c_probe(uint8_t addr) {
    /*Probes for a device at the addr*/
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    /*Reads from device with addr data in register into data with passed length*/
    assert(len > 0);
    memset(data, 0, len); //zero out buffer

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    if (len > 1)
        i2c_master_read(cmd, data, len-1, I2C_MASTER_ACK); //all but last data point
    i2c_master_read_byte(cmd, data+len-1, I2C_MASTER_LAST_NACK); //last data bit
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    /*Sets the value of reg to the passed value*/
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, val, I2C_MASTER_LAST_NACK);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t i2c_write_data(uint8_t addr, void *data, uint8_t len) {
    /*writes everyting in data to the device at addr*/
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, len, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t i2c_read_data(uint8_t addr, void *data, uint8_t len) {
    /*Reads some arbitrary data from device at addr*/
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    if (len > 1)
        i2c_master_read(cmd, data, len-1, I2C_MASTER_ACK); //all but last data point
    i2c_master_read_byte(cmd, data+len-1, I2C_MASTER_LAST_NACK); //last data bit
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpl_start_measurement() {
    uint8_t *data = malloc(1);
    esp_err_t ret = i2c_read_reg(MPL_ADDR, MPL_CTRL_REG1, data, 1 );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read [%d] | Ret val=%d / %s", data[0], ret,  esp_err_to_name(ret));
        free(data);
        return ret; 
    }
    data[0] &= ~(1<<1); //Clear OST Bit
    ret = i2c_write_reg(MPL_ADDR, MPL_CTRL_REG1, data[0] );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write [%d] | Ret val=%d / %s", data[0], ret,  esp_err_to_name(ret));
        free(data);
        return ret;
    }

    ret = i2c_read_reg(MPL_ADDR, MPL_CTRL_REG1, data, 1 );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read2 [%d] Ret val=%d / %s", data[0], ret, esp_err_to_name(ret));
        free(data);
        return ret;
    }
    data[0] |= (1<<1); //Set OST bit
    ret = i2c_write_reg(MPL_ADDR, MPL_CTRL_REG1, data[0] );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write2 [%d] Ret val=%d / %s", data[0], ret, esp_err_to_name(ret));
    }
    free(data);
    return ret;
}
esp_err_t mpl_wait_for_ready() {
    /*blocks until the status flag is set, or around 12 cycles of 50ms*/
    esp_err_t err = mpl_start_measurement();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ERROR");
        return err;
    }
        
    int counter = 0;
    uint8_t *data = malloc(1);

    while (counter++ < 12) {
        if ((err = i2c_read_reg(MPL_ADDR, MPL_STATUS, data, 1)) != ESP_OK)
            continue;
        err = ESP_ERR_INVALID_STATE;
        if (data[0] & (1 << 2)) {
            err = ESP_OK;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    free(data);
    return err;
}
esp_err_t mpl_read_data(float *pressure, float *temperature) {
    esp_err_t err;
    if ((err = mpl_wait_for_ready()) != ESP_OK)
        return err;

    uint8_t *data = malloc(5);
    *pressure = 0;
    *temperature = -99.99;

    err = i2c_read_reg(MPL_ADDR, MPL_OUT_P_MSB, data, 5);

	data[2] &= 0B00110000; //Bits 5/4 represent the fractional component
	data[2] >>= 4; //Get it right aligned
	*pressure = ((float)(((long)data[0]<<16 | (long)data[1]<<8 | (long)data[2]) >> 6) + (float)data[2]/4.0) / 100.0;
    
    int16_t t = (data[3] << 8  | data[4]) >> 4;
    if (t & 0x800) t |= 0xF000;
    *temperature = (float)t / 16.0;
    
    free(data);
    return err;
}

esp_err_t mpl_init() {
    while (i2c_probe(MPL_ADDR) != ESP_OK ) {
        ESP_LOGW(TAG, "Waiting for MPL Bus to be ready");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    //configure registers
    return i2c_write_reg(MPL_ADDR, MPL_CTRL_REG1, 0x38) ||
    i2c_write_reg(MPL_ADDR, MPL_PT_DATA_CFG, 0x07) || 
    i2c_write_reg(MPL_ADDR, MPL_CTRL_REG1, 0x39);
}

esp_err_t si7021_init() {
    while (i2c_probe(SI7021_ADDR) != ESP_OK ) {
        ESP_LOGW(TAG, "Waiting for SI7021 to be ready");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    uint8_t data;
    while (i2c_read_reg(SI7021_ADDR, SI7021_READRHT_REG_CMD,&data, 1) != ESP_OK && data != 0x3A) {
        ESP_LOGW(TAG, "SI7021 not responding");
        data = SI7021_RESET_CMD;
        i2c_write_data(SI7021_ADDR, &data, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_OK;
}

esp_err_t si7021_read_data(float *humidity, float *temperature) {
    esp_err_t err;
    uint8_t cmds[] = {SI7021_MEASRH_NOHOLD_CMD, SI7021_MEASTEMP_NOHOLD_CMD};
    uint8_t data[3];
    *humidity = -99, *temperature = -99;

    //start measuring RH
    if ((err = i2c_write_data(SI7021_ADDR, cmds, 1)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(50)); //wait 50ms

    if ((err = i2c_read_data(SI7021_ADDR, data, 3)) != ESP_OK) return err;
    *humidity = 125 * (data[0] << 8 | data[1]) / 65546 - 6; //data[2] is a CRC

    //start measuring T
    if ((err = i2c_write_data(SI7021_ADDR, cmds+1, 1)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(25)); //wait 25ms

    err = i2c_read_data(SI7021_ADDR, data, 3);
    *temperature = 175.72 * (data[0] << 8 | data[1]) / 65546 - 46.85; //data[2] is a CRC
    return err;
}

void i2c_data_init(void *parameter) {
    //init i2c bus
    i2c_config_t *conf = (i2c_config_t *) parameter;
    ESP_ERROR_CHECK(init_i2c_bus(conf));

    si7021_init();
    mpl_init();
    esp_err_t err;
    float a, b;
    char * buf = malloc(1024);
    
    for ( ;; ) {
        err = mpl_read_data(&a, &b);
        weather_data.p = update_value_f(a, err);
        weather_data.pt = update_value_f(b, err);

        err = si7021_read_data(&a, &b);
        weather_data.rh = update_value_f(a, err);
        weather_data.rht = update_value_f(b, err);
        
        int len = json_wx_data_t(buf, 1024, weather_data);
        mqtt_publish_msg(CONFIG_MQTT_TOPIC_WEATHER, buf, len );
        // ESP_LOGI(TAG, "%s", buf);
        
        //Check for odd I2C errors on Barometer
        switch (weather_data.p.error) {
            case ESP_OK:
                // ESP_LOGI(TAG, "Pressure = %f\t Temp = %f", weather_data.p.sample.f, weather_data.pt.sample.f);
                break;
            case ESP_ERR_INVALID_STATE:
                mpl_init();
                // fall through
            default:
                ESP_LOGE(TAG, "Unable to read data - %d | %s", weather_data.p.error, esp_err_to_name(weather_data.p.error));
        }

        //Check for odd I2C errors on Barometer
        switch (weather_data.rh.error) {
            case ESP_OK:
                // ESP_LOGI(TAG, "Humidity = %f\t Temp = %f", weather_data.rh.sample.f,  weather_data.rht.sample.f);
                break;
            case ESP_ERR_INVALID_STATE:
                si7021_init();
                // fall through
            default:
                ESP_LOGE(TAG, "Unable to read data - %d | %s", weather_data.p.error, esp_err_to_name(weather_data.p.error));
        }        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    free(buf);
}

void wxstation_init(void) {
    weather_data.p = update_value_f(0, ESP_OK);
    weather_data.pt = update_value_f(0, ESP_OK);
    weather_data.rh = update_value_f(0, ESP_OK);
    weather_data.rht = update_value_f(0, ESP_OK);

    TaskHandle_t tsk;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency,
    };
    
    xTaskCreate(i2c_data_init,"i2c_data_init",2048, (void * const)&conf, 1, &tsk);

    ESP_LOGI(TAG, "I2C connected");
}

