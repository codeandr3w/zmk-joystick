/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_push_key_press

#include <device.h>
#include <drivers/behavior.h>
#include <logging/log.h>
#include <stdlib.h>

#include <drivers/sensor.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int behavior_sensor_push_key_press_init(const struct device *dev) { return 0; };

struct behavior_sensor_push_key_press_data {
  bool pressed;
  int pressed_keycode;
};

struct behavior_sensor_push_key_press_config {
  int min_push;
};

static int on_sensor_binding_triggered(struct zmk_behavior_binding *binding,
                                       const struct device *sensor, int64_t timestamp) {
    struct sensor_value value;
    int err;
    uint32_t keycode;
    int pressed = false;
    
    const struct device *dev = device_get_binding(binding->behavior_dev);
    struct behavior_sensor_push_key_press_data *data = dev->data;
    struct behavior_sensor_push_key_press_config *config = dev->config;
    

    err = sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &value);

    if (err) {
        LOG_WRN("Failed to ge sensor rotation value: %d", err);
        return err;
    }

    LOG_DBG("push left keycode 0x%02X right keycode 0x%02X, value %d, old_pressed %d", binding->param1, binding->param2, value.val1, data->pressed);

    if (!pressed && data->pressed && data->pressed_keycode!=binding->param1 && data->pressed_keycode!=binding->param2) {
        // nothing is pressed, but another binding is pressed, so stick to that one
        pressed = data->pressed;
        keycode = data->pressed_keycode;
    } else if (!data->pressed && (abs(value.val1) < config->min_push)) {
        // not pushed enough to start a press
    } else if (value.val1 > 0) {
        keycode = binding->param1;
        pressed = true;
    } else if (value.val1 < 0) {
        keycode = binding->param2;
        pressed = true;
    }
    
    if (pressed && data->pressed) {
        if (data->pressed_keycode != keycode) {
            // pressed a different key from last time
            ZMK_EVENT_RAISE(zmk_keycode_state_changed_from_encoded(data->pressed_keycode, false, timestamp));
            err = ZMK_EVENT_RAISE(zmk_keycode_state_changed_from_encoded(keycode, true, timestamp));
            LOG_DBG("RELEASE SEND %d %d", data->pressed_keycode, keycode);
        }
        // else: no change
    } else if (pressed) {
        // newly pressed key
        err = ZMK_EVENT_RAISE(zmk_keycode_state_changed_from_encoded(keycode, true, timestamp));
        LOG_DBG("SEND %d", keycode);
    } else if (data->pressed) {
        // newly released key
        err = ZMK_EVENT_RAISE(zmk_keycode_state_changed_from_encoded(data->pressed_keycode, false, timestamp));
        LOG_DBG("RELEASE %d", data->pressed_keycode);
    }

    data->pressed = pressed;
    data->pressed_keycode = keycode;

    return err;
}

static const struct behavior_driver_api behavior_sensor_push_key_press_driver_api = {
    .sensor_binding_triggered = on_sensor_binding_triggered};

#define KP_INST(n)                                                                                 \
    static struct behavior_sensor_push_key_press_data sensor_push_key_press_data_##n; 		   \
    static struct behavior_sensor_push_key_press_config sensor_push_key_press_config_##n = { 		   \
        .min_push = DT_INST_PROP(n, min_push), \
        }; \
    DEVICE_DT_INST_DEFINE(n, behavior_sensor_push_key_press_init, NULL,   \
                          &sensor_push_key_press_data_##n,                \
                          &sensor_push_key_press_config_##n,                \
                          APPLICATION, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                     \
                          &behavior_sensor_push_key_press_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
