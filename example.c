#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"

#include "adxl343.h" // WIP, comment out this line for current builds

static char TAG[] = "main";

void app_main()
{
    ADXL343_cfg acc1 = {0};
    acc1.interface.interface_type = ADXL343_I2C;
    acc1.interface.interface.i2c = (ADXL343I2CSettings) {
        .scl_pin = 5,
        .sda_pin = 6,
        .addr = 0x53,
        .port_num = 0,
        .freq = 100000
    };
    acc1.sensor_id = 1;
    ADXL343_initialise(&acc1);

    int16_t x, y, z;
    ESP_LOGI(TAG, "Got ID: %u", ADXL343_get_device_id(&acc1));

    while(1)
    {
        ADXL343_get_x_y_z(&acc1, &x, &y, &z);
        ESP_LOGI(TAG, "Raw X: %d, Y: %d, Z: %d", x, y, z);
        ESP_LOGI(TAG, "X: %fm/s^2, Y: %fm/s^2, Z: %fm/s^2", x*ADXL343_MG2G_MULTIPLIER*ADXL343_GRAVITY, y*ADXL343_MG2G_MULTIPLIER*ADXL343_GRAVITY, z*ADXL343_MG2G_MULTIPLIER*ADXL343_GRAVITY);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}