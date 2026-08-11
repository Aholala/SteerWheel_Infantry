#include "app_imu.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"

#include <math.h>

bsp_status_t app_imu_init(app_imu_t *me, const app_imu_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->sensor == NULL) ||
        (config->accelerometer_correction_gain < 0.0F) ||
        (config->accelerometer_correction_gain > 1.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_imu_t){
        .config = *config,
        .initialized = true,
    };
    return BSP_STATUS_OK;
}

void app_imu_update(app_imu_t *me, float delta_time_s)
{
    const module_bmi088_process_data_t *data;
    float accelerometer_pitch_rad;
    float accelerometer_roll_rad;
    float gain;

    if ((me == NULL) || !me->initialized)
    {
        return;
    }
    if (module_bmi088_read(me->config.sensor) != MODULE_BMI088_STATUS_OK)
    {
        me->snapshot.valid = false;
        app_exchange_publish_imu(&me->snapshot);
        return;
    }
    data = module_bmi088_get_data(me->config.sensor);
    if ((data == NULL) || !data->is_valid)
    {
        me->snapshot.valid = false;
        app_exchange_publish_imu(&me->snapshot);
        return;
    }

    me->snapshot.roll_rad += data->angular_velocity_rad_per_s[0] * delta_time_s;
    me->snapshot.pitch_rad += data->angular_velocity_rad_per_s[1] * delta_time_s;
    me->snapshot.yaw_rad += data->angular_velocity_rad_per_s[2] * delta_time_s;
    accelerometer_roll_rad =
        atan2f(data->acceleration_m_per_s2[1], data->acceleration_m_per_s2[2]);
    accelerometer_pitch_rad =
        atan2f(-data->acceleration_m_per_s2[0],
               sqrtf((data->acceleration_m_per_s2[1] * data->acceleration_m_per_s2[1]) +
                     (data->acceleration_m_per_s2[2] * data->acceleration_m_per_s2[2])));
    gain = me->config.accelerometer_correction_gain;
    me->snapshot.roll_rad =
        ((1.0F - gain) * me->snapshot.roll_rad) + (gain * accelerometer_roll_rad);
    me->snapshot.pitch_rad =
        ((1.0F - gain) * me->snapshot.pitch_rad) + (gain * accelerometer_pitch_rad);
    me->snapshot.angular_velocity_rad_per_s[0] = data->angular_velocity_rad_per_s[0];
    me->snapshot.angular_velocity_rad_per_s[1] = data->angular_velocity_rad_per_s[1];
    me->snapshot.angular_velocity_rad_per_s[2] = data->angular_velocity_rad_per_s[2];
    me->snapshot.sample_count = data->sample_count;
    me->snapshot.valid = true;
    app_exchange_publish_imu(&me->snapshot);
}
