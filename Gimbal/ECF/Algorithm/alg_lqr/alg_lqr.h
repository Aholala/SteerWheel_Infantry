/**
 * @file alg_lqr.h
 * @brief 固定增益线性二次型状态反馈控制器
 * @note 增益矩阵由离线工具生成；车载端只执行状态反馈与限幅。
 */
#ifndef ALG_LQR_H
#define ALG_LQR_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ALG_LQR_STATUS_OK = 0,
    ALG_LQR_STATUS_INVALID_ARGUMENT,
    ALG_LQR_STATUS_OUT_OF_RANGE,
    ALG_LQR_STATUS_NOT_INITIALIZED,
    ALG_LQR_STATUS_NUMERICAL_ERROR
} alg_lqr_status_t;

typedef struct
{
    size_t state_dimension;
    size_t control_dimension;
    const float *gain_matrix;
    const float *control_min;
    const float *control_max;
} alg_lqr_config_t;

typedef struct
{
    alg_lqr_config_t config;
    bool is_initialized;
} alg_lqr_t;

alg_lqr_status_t alg_lqr_init(alg_lqr_t *me, const alg_lqr_config_t *config);

/**
 * @brief 计算 output = feedforward - K * (state - reference)
 * @param reference 可为 NULL，表示零参考状态
 * @param feedforward 可为 NULL，表示零前馈
 */
alg_lqr_status_t alg_lqr_update(const alg_lqr_t *me, const float *reference,
                                const float *state, const float *feedforward,
                                float *output);

#ifdef __cplusplus
}
#endif

#endif
