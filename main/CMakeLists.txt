idf_component_register(
    SRCS
        "main.c"
        "line_math.c"
        "line_sensors.c"
        "motors.c"
        "button.c"
        "status_led.c"
        "power_sense.c"
        "calibration_store.c"
        "calibrate.c"
        "follower.c"
        "console_ctrl.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc nvs_flash esp_timer freertos
)
