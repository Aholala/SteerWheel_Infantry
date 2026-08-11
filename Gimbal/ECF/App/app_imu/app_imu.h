#ifndef APP_IMU_H
#define APP_IMU_H

#include "bsp_common.h"
#include "app_types.h"
#include "module_bmi088.h"

#include <stdbool.h>

typedef struct
{
    module_bmi088_t *sensor;
    float accelerometer_correction_gain;
} app_imu_config_t;

typedef struct
{
    app_imu_config_t config;
    app_imu_snapshot_t snapshot;
    bool initialized;
} app_imu_t;

bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config);
void app_imu_update(app_imu_t *me, float delta_time_s);

#endif
