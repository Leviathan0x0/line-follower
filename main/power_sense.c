#include "power_sense.h"

#include "board_config.h"

#include "esp_log.h"

static const char *TAG = "power";

#if BOARD_BATTERY_SENSE_ENABLE

#include <stdint.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_channel;
static bool s_ready;
static bool s_have_cali;
static int s_mv = -1;
static int s_low_count;
static int s_prescale;

static int pin_millivolts(int raw)
{
    int mv = 0;
    if (s_have_cali && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
        return mv;
    }
    return (raw * 3100) / 4095;
}

void power_sense_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &s_adc) != ESP_OK) {
        ESP_LOGW(TAG, "battery ADC unit failed");
        return;
    }

    adc_unit_t unit = ADC_UNIT_2;
    if (adc_oneshot_io_to_channel(BOARD_VBAT_GPIO, &unit, &s_channel) != ESP_OK || unit != ADC_UNIT_2) {
        ESP_LOGW(TAG, "GPIO %d is not an ADC2 battery pin", BOARD_VBAT_GPIO);
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    if (adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "battery channel config failed");
        return;
    }

    adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id = ADC_UNIT_2,
        .chan = s_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_cali) == ESP_OK) {
        s_have_cali = true;
    } else {
        ESP_LOGW(TAG, "battery ADC calibration unavailable; cutoff is approximate");
    }

    s_ready = true;
    ESP_LOGI(TAG, "pack sense on GPIO %d, cutoff %d mV, divider %d/%d",
             BOARD_VBAT_GPIO, BOARD_VBAT_CUTOFF_MV, BOARD_VBAT_R_TOP, BOARD_VBAT_R_BOT);
}

void power_sense_poll(void)
{
    if (!s_ready) {
        return;
    }
    if (++s_prescale < 10) {
        return;
    }
    s_prescale = 0;

    int raw = 0;
    if (adc_oneshot_read(s_adc, s_channel, &raw) != ESP_OK) {
        return;
    }
    int pin_mv = pin_millivolts(raw);
    int pack_mv = (int)(((int64_t)pin_mv * (BOARD_VBAT_R_TOP + BOARD_VBAT_R_BOT)) / BOARD_VBAT_R_BOT);
    s_mv = pack_mv;
    if (pack_mv < BOARD_VBAT_CUTOFF_MV) {
        if (s_low_count < 1000) {
            s_low_count++;
        }
    } else {
        s_low_count = 0;
    }
}

int power_sense_mv(void)
{
    return s_mv;
}

bool power_sense_low(void)
{
    return s_ready && s_low_count >= BOARD_VBAT_LOW_SAMPLES;
}

#else

void power_sense_init(void)
{
    ESP_LOGI(TAG, "pack sensing off (BOARD_BATTERY_SENSE_ENABLE is 0)");
}

void power_sense_poll(void)
{
}

int power_sense_mv(void)
{
    return -1;
}

bool power_sense_low(void)
{
    return false;
}

#endif
