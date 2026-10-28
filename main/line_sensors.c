#include "line_sensors.h"

#include "board_config.h"

#include <stdlib.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "sensors";

_Static_assert(BOARD_SENSOR_COUNT == LINE_SENSOR_COUNT, "sensor count mismatch");

static const gpio_num_t s_gpio[LINE_SENSOR_COUNT] = {
    BOARD_SENSOR0_GPIO,
    BOARD_SENSOR1_GPIO,
    BOARD_SENSOR2_GPIO,
    BOARD_SENSOR3_GPIO,
    BOARD_SENSOR4_GPIO,
};

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_channel[LINE_SENSOR_COUNT];
static sensor_cal_t s_cal;

static int average_channel(adc_channel_t channel)
{
    int sum = 0;
    for (int i = 0; i < BOARD_ADC_SAMPLES; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc, channel, &raw);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "adc read failed: %s", esp_err_to_name(err));
            return sum / (i == 0 ? 1 : i);
        }
        sum += raw;
    }
    return sum / BOARD_ADC_SAMPLES;
}

esp_err_t line_sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        adc_unit_t unit = ADC_UNIT_1;
        adc_channel_t channel = ADC_CHANNEL_0;
        ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(s_gpio[i], &unit, &channel),
                            TAG, "gpio %d is not an adc pin", s_gpio[i]);
        if (unit != ADC_UNIT_1) {
            ESP_LOGE(TAG, "gpio %d is not on ADC1", s_gpio[i]);
            return ESP_ERR_INVALID_ARG;
        }
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, channel, &chan_cfg),
                            TAG, "config gpio %d", s_gpio[i]);
        s_channel[i] = channel;
        ESP_LOGI(TAG, "sensor %d -> GPIO %d (ADC1 ch %d)", i, s_gpio[i], channel);
    }

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        s_cal.white[i] = 700;
        s_cal.black[i] = 2400;
    }

    int discard[LINE_SENSOR_COUNT];
    for (int n = 0; n < 4; n++) {
        line_sensors_read_raw(discard);
    }
    return ESP_OK;
}

void line_sensors_read_raw(int raw[LINE_SENSOR_COUNT])
{
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        raw[i] = (s_adc != NULL) ? average_channel(s_channel[i]) : 0;
    }
}

void line_sensors_set_calibration(const sensor_cal_t *cal)
{
    if (cal != NULL) {
        s_cal = *cal;
    }
}

void line_sensors_get_calibration(sensor_cal_t *cal)
{
    if (cal != NULL) {
        *cal = s_cal;
    }
}

bool line_sensors_cal_valid(const sensor_cal_t *cal)
{
    if (cal == NULL) {
        return false;
    }
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        int span = abs(cal->black[i] - cal->white[i]);
        if (span < BOARD_CAL_MIN_SPAN) {
            return false;
        }
        if (cal->white[i] < 0 || cal->black[i] < 0 || cal->white[i] > 4095 || cal->black[i] > 4095) {
            return false;
        }
    }
    return true;
}

int line_sensors_gpio(int physical_index)
{
    if (physical_index < 0 || physical_index >= LINE_SENSOR_COUNT) {
        return -1;
    }
    return (int)s_gpio[physical_index];
}

void line_sensors_read(line_reading_t *out)
{
    if (out == NULL) {
        return;
    }

    int physical[LINE_SENSOR_COUNT] = {0};
    line_sensors_read_raw(physical);

    for (int logical = 0; logical < LINE_SENSOR_COUNT; logical++) {
        int physical_index = logical;
#if BOARD_SENSOR_REVERSE
        physical_index = LINE_SENSOR_COUNT - 1 - logical;
#endif
        out->raw[logical] = physical[physical_index];
        out->value[logical] = line_normalize(physical[physical_index],
                                             s_cal.white[physical_index],
                                             s_cal.black[physical_index]);
    }

    out->seen = line_centroid(out->value, BOARD_BLACK_THRESHOLD, &out->position, &out->total);
}
