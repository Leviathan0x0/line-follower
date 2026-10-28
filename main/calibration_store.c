#include "calibration_store.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "store";
static const char *NVS_NS = "follower";

#define CAL_MAGIC 0x314C4143u
#define GAIN_MAGIC 0x314E4147u

typedef struct {
    uint32_t magic;
    int32_t white[LINE_SENSOR_COUNT];
    int32_t black[LINE_SENSOR_COUNT];
} cal_blob_t;

typedef struct {
    uint32_t magic;
    float base_duty;
    float kp;
    float kd;
    float ki;
    float slowdown;
} gain_blob_t;

bool calibration_store_load(sensor_cal_t *cal)
{
    if (cal == NULL) {
        return false;
    }

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    cal_blob_t blob;
    size_t size = sizeof(blob);
    esp_err_t err = nvs_get_blob(handle, "cal", &blob, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(blob) || blob.magic != CAL_MAGIC) {
        return false;
    }

    sensor_cal_t loaded;
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        loaded.white[i] = (int)blob.white[i];
        loaded.black[i] = (int)blob.black[i];
    }
    if (!line_sensors_cal_valid(&loaded)) {
        ESP_LOGW(TAG, "stored calibration failed the span check");
        return false;
    }

    *cal = loaded;
    return true;
}

esp_err_t calibration_store_save(const sensor_cal_t *cal)
{
    if (cal == NULL || !line_sensors_cal_valid(cal)) {
        return ESP_ERR_INVALID_ARG;
    }

    cal_blob_t blob = {.magic = CAL_MAGIC};
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        blob.white[i] = (int32_t)cal->white[i];
        blob.black[i] = (int32_t)cal->black[i];
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, "cal", &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "calibration saved");
    }
    return err;
}

bool calibration_store_load_gains(tracker_cfg_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    gain_blob_t blob;
    size_t size = sizeof(blob);
    esp_err_t err = nvs_get_blob(handle, "gains", &blob, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(blob) || blob.magic != GAIN_MAGIC) {
        return false;
    }
    if (!isfinite(blob.base_duty) || blob.base_duty < 0.0f || blob.base_duty > 1024.0f ||
        !isfinite(blob.kp) || blob.kp < 0.0f || blob.kp > 4000.0f ||
        !isfinite(blob.kd) || blob.kd < 0.0f || blob.kd > 400.0f ||
        !isfinite(blob.ki) || blob.ki < 0.0f || blob.ki > 2000.0f ||
        !isfinite(blob.slowdown) || blob.slowdown < 0.0f || blob.slowdown > 1.0f) {
        ESP_LOGW(TAG, "stored gains were out of range");
        return false;
    }

    cfg->base_duty = blob.base_duty;
    cfg->kp = blob.kp;
    cfg->kd = blob.kd;
    cfg->ki = blob.ki;
    cfg->slowdown = blob.slowdown;
    ESP_LOGI(TAG, "stored gains loaded");
    return true;
}

esp_err_t calibration_store_save_gains(const tracker_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gain_blob_t blob = {
        .magic = GAIN_MAGIC,
        .base_duty = cfg->base_duty,
        .kp = cfg->kp,
        .kd = cfg->kd,
        .ki = cfg->ki,
        .slowdown = cfg->slowdown,
    };

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, "gains", &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "gains saved");
    }
    return err;
}

esp_err_t calibration_store_erase_gains(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(handle, "gains");
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}
