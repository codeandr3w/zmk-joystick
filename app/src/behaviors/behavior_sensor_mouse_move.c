/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_mouse_move

#include <device.h>
#include <drivers/behavior.h>
#include <logging/log.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/mouse_move_state_changed.h>
#include <zmk/mouse.h>
#include <drivers/sensor.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_sensor_mouse_move_config {
  int speed, dimension;
};

static int behavior_sensor_mouse_move_init(const struct device *dev) { return 0; };

static int on_sensor_binding_triggered(struct zmk_behavior_binding *binding,
                                       const struct device *sensor, int64_t timestamp) {
    struct sensor_value value;
    int err;
    uint32_t keycode;
    
    const struct device *dev = device_get_binding(binding->behavior_dev);
    struct behavior_sensor_mouse_move_config *config = dev->config;
    

    err = sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &value);

    if (err) {
        LOG_WRN("Failed to ge sensor rotation value: %d", err);
        return err;
    }

    LOG_DBG("push push dimension %d, value %d, input dimension %d", binding->param1, value.val1, config->dimension);


    // need to move mouse by value in dimension binding->param1
    int x=0, y=0;
    int move = 0;
    if (value.val1 != 0) {
        move = (config->speed * value.val1 * value.val1) >> 20;
        move += 1;
        if (value.val1 < 0) {
            move = -move;
        }
    }
    if (binding->param1 == 0) {
        x += move;
    } else if (binding->param1 == 1) {
        y += move;
    }
    zmk_hid_mouse_movement_set (x, y);
    zmk_endpoints_send_mouse_report ();
    return err;
}

static const struct behavior_driver_api behavior_sensor_mouse_move_driver_api = {
    .sensor_binding_triggered = on_sensor_binding_triggered};

#define KP_INST(n)                                                                                 \
    static struct behavior_sensor_mouse_move_config sensor_mouse_move_config_##n = { 		   \
        .speed = DT_INST_PROP(n, speed), \
        .dimension = DT_INST_PROP(n, dimension), \
        }; \
    DEVICE_DT_INST_DEFINE(n, behavior_sensor_mouse_move_init, NULL,   \
                          NULL,                                             \
                          &sensor_mouse_move_config_##n,                \
                          APPLICATION, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                     \
                          &behavior_sensor_mouse_move_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
