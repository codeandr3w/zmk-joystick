/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_var_key_press

#include <device.h>
#include <drivers/behavior.h>
#include <logging/log.h>
#include <stdlib.h>

#include <drivers/sensor.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define NUM_DIM 2

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int behavior_sensor_var_key_press_init(const struct device *dev) { return 0; };

struct behavior_sensor_var_key_press_config {
    int dimensions;
    int time;
    int key_diff [NUM_DIM];
};

struct behavior_sensor_var_key_press_data {
    int timer [NUM_DIM];
    int delay [NUM_DIM];
    int last_keycode [NUM_DIM];
}; 

static int on_sensor_binding_triggered(struct zmk_behavior_binding *binding,
                                       const struct device *sensor, int64_t timestamp) {
    struct sensor_value value [NUM_DIM];
    int err;
    uint32_t keycode = 0;
    
    const struct device *dev = device_get_binding(binding->behavior_dev);
    struct behavior_sensor_var_key_press_data *data = dev->data;
    struct behavior_sensor_var_key_press_config *config = dev->config;

    for (int dim=0; dim<config->dimensions; dim++) {
        
        err = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_X+dim, &(value [dim]));
        if (err) {
            value [dim].val1 = 0;
        }

        uint32_t up_keycode = (&binding->param1) [dim];
        uint32_t down_keycode = up_keycode + config->key_diff [dim];

        LOG_DBG("var %d, dev:%p cfg: %p up keycode 0x%02X down keycode 0x%02X, val=%d, timer=%d, delay=%d", 
            dim, dev, config, up_keycode, down_keycode, value [dim].val1, data->timer [dim], data->delay [dim]);

        if (value [dim].val1 > 0) {
            keycode = down_keycode;
        } else if (value [dim].val1 < 0) {
            keycode = up_keycode;
        } else {
            data->last_keycode [dim] = -1;
            data->delay [dim] = 0;
            continue;
        }
    
        if (data->last_keycode [dim] == keycode) {
            if (data->delay [dim] > 0) {
                --data->delay [dim];
                continue;
            }
            data->timer [dim] += abs(value [dim].val1);
        } else {
            data->timer [dim] = config->time;
            data->last_keycode [dim] = keycode;
            data->delay [dim] = 20; // this should be the default keyboard key repeat delay
        }
    
        if(data->timer [dim] >= config->time) {

            data->timer [dim] -= config->time;

            ZMK_EVENT_RAISE(zmk_keycode_state_changed_from_encoded(keycode, true, timestamp));

            LOG_DBG("SEND %d, timer=%d", keycode, data->timer);
              // TODO: Better way to do this?
            k_msleep(5);

            ZMK_EVENT_RAISE(zmk_keycode_state_changed_from_encoded(keycode, false, timestamp));
        }
    }
    return 0;
}

static const struct behavior_driver_api behavior_sensor_var_key_press_driver_api = {
    .sensor_binding_triggered = on_sensor_binding_triggered};

#define KP_INST(n)                                                                                 \
    static struct behavior_sensor_var_key_press_config sensor_var_key_press_config_##n = { 		   \
        .dimensions = DT_INST_PROP(n, dimensions), \
        .time = DT_INST_PROP(n, time), \
        .key_diff [0] = DT_INST_PROP(n, key_diff_x), \
        .key_diff [1] = DT_INST_PROP(n, key_diff_y), \
        }; \
    static struct behavior_sensor_var_key_press_data sensor_var_key_press_data_##n; 		   \
    DEVICE_DT_INST_DEFINE(n, behavior_sensor_var_key_press_init, NULL, \
                          &sensor_var_key_press_data_##n, 					\
                          &sensor_var_key_press_config_##n, APPLICATION, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                     \
                          &behavior_sensor_var_key_press_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
