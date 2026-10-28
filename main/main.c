#include "button.h"
#include "console_ctrl.h"
#include "follower.h"
#include "line_sensors.h"
#include "motors.h"
#include "power_sense.h"
#include "status_led.h"

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "app";

void app_main(void)
{
    /* Brake before flash init so a slow boot cannot leave the L298N enables floating. */
    ESP_ERROR_CHECK(motors_init());
    status_led_init();
    button_init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(line_sensors_init());
    power_sense_init();

    ESP_LOGI(TAG, "ESP32-S3 line follower: 5 analog IR sensors, L298N, two N20 motors");
    follower_start();
    console_start();
}
