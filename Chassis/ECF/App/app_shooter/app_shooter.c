#include "app_shooter.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"

bsp_status_t app_shooter_init(app_shooter_t *me, const app_shooter_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->shooter == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_shooter_t){
        .config = *config,
        .initialized = true,
    };
    return BSP_STATUS_OK;
}

void app_shooter_update(app_shooter_t *me, float delta_time_s)
{
    app_shooter_command_t command;
    app_gimbal_feedback_t gimbal;
    app_shooter_feedback_t feedback;
    const module_board_comm_shooter_process_data_t *remote_feedback = NULL;

    if ((me == NULL) || !me->initialized)
    {
        return;
    }
    app_exchange_read_shooter_command(&command);
    app_exchange_read_gimbal_feedback(&gimbal);
    if (!me->config.shooter->has_local_friction && (me->config.board_comm != NULL) &&
        me->config.board_comm->shooter_online)
    {
        remote_feedback = module_board_comm_get_shooter(me->config.board_comm);
        (void)module_shooter_set_external_friction_ready(
            me->config.shooter,
            (remote_feedback != NULL) && remote_feedback->friction_ready);
    }
    if (command.friction_enabled &&
        (module_shooter_get_state(me->config.shooter) == MODULE_SHOOTER_STATE_DISABLED))
    {
        (void)module_shooter_enable(me->config.shooter);
    }
    else if (!command.friction_enabled &&
             (module_shooter_get_state(me->config.shooter) != MODULE_SHOOTER_STATE_DISABLED))
    {
        (void)module_shooter_disable(me->config.shooter);
    }
    (void)module_shooter_set_friction(me->config.shooter, command.friction_enabled,
                                      command.friction_velocity_rad_per_s);
    if (me->config.shooter->has_local_feeder && command.fire_requested &&
        !me->previous_fire_request)
    {
        (void)module_shooter_request_shots(me->config.shooter, 1U);
    }
    me->previous_fire_request = command.fire_requested;

    if (me->config.shooter->has_local_feeder && command.automatic_fire_enabled)
    {
        const module_shooter_fire_control_input_t fire_control = {
            .automatic_fire_enabled = true,
            .tracking_ready = gimbal.target_locked,
            .referee_allows_fire = true,
        };
        (void)module_shooter_update_fire_control(me->config.shooter, &fire_control,
                                                 delta_time_s);
    }
    (void)module_shooter_update(me->config.shooter, delta_time_s);

    feedback.state = (uint8_t)module_shooter_get_state(me->config.shooter);
    feedback.jam_retry_count = module_shooter_get_jam_retry_count(me->config.shooter);
    feedback.friction_ready = module_shooter_get_friction_ready(me->config.shooter);
    feedback.fire_permission = module_shooter_get_fire_permission(me->config.shooter);
    app_exchange_publish_shooter_feedback(&feedback);
    if (me->config.board_comm != NULL)
    {
        const module_board_comm_shooter_process_data_t board_data = {
            .state = feedback.state,
            .jam_retry_count = feedback.jam_retry_count,
            .friction_ready = feedback.friction_ready,
            .fire_permission = feedback.fire_permission,
        };
        if (module_board_comm_send_shooter(me->config.board_comm, &board_data) !=
            MODULE_BOARD_COMM_STATUS_OK)
        {
            bsp_error_record(BSP_STATUS_IO_ERROR, "send_shooter", 0);
        }
    }
}
