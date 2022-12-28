/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT joystick

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <stdlib.h>
#include <pm/device.h>
#include "../../../include/drivers/ext_power.h"

#include "joystick.h"

static void zmk_joy_work(struct k_work *work);

LOG_MODULE_REGISTER(JOYSTICK, CONFIG_ZMK_LOG_LEVEL);

static const struct device *ext_power;

static void joy_get_state(const struct device *dev) {
    struct joy_data *drv_data = dev->data;
    const struct joy_config *drv_cfg = dev->config;
    int disable_power = 0;

    if (ext_power != NULL) {
        int power = ext_power_get(ext_power);
        if (!power) {
            // power is off but must be turned on for ADC
            int rc = ext_power_enable(ext_power);
            if (rc != 0) {
                LOG_ERR("Unable to enable EXT_POWER: %d", rc);
            }
            k_sleep(K_MSEC(10)); // wait for charge to build up so ADC is ready
            disable_power = 1;
        }
    }

    for (int axis=0; axis<drv_data->num_axis; axis++) {
        if (drv_data->adc [axis] == 0)
            continue;
        int rc = adc_read(drv_data->adc [axis], &(drv_data->as [axis]));
        int32_t val = drv_data->adc_raw [axis];
        if (val > 4096) 
            val = 4096;
        if (rc != 0) {
	    LOG_DBG("Joy failed to read ADC%d: %d", axis, rc);
        }
        val -= drv_data->zero_value [axis];
        if (drv_cfg->reverse) {
            val = -val;
        }
        drv_data->delta [axis] = val - drv_data->value [axis];
        drv_data->value [axis] = val;
        drv_data->as [axis].calibrate = false;
    }
    
    if (disable_power) {
        int rc = ext_power_disable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to disable EXT_POWER: %d", rc);
        }
    }    
}

static int joy_sample_fetch(const struct device *dev, enum sensor_channel chan) {
// unnecessary because we use timers to read the joystick
    return 0;
}

static int joy_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct joy_data *drv_data = dev->data;
    const struct joy_config *drv_cfg = dev->config;
    
    if (chan == SENSOR_CHAN_ROTATION) {
        // This is here to behave like a rotational encoder
        int value = drv_data->value [0];
        val->val1 = 0;
        val->val2 = 0;
        if (value >= drv_data->last_rotate [0] + drv_cfg->resolution) {
            drv_data->last_rotate [0] += drv_cfg->resolution;
            val->val1 = 1;
        } else if (value <= drv_data->last_rotate [0] - drv_cfg->resolution) {
            drv_data->last_rotate [0] -= drv_cfg->resolution;
            val->val1 = -1;
        }
        return 0;
    } else if (chan >= SENSOR_CHAN_ACCEL_X && chan < (SENSOR_CHAN_ACCEL_X+drv_data->num_axis)) {
        // This is here to handle special joystick behaviours
        int value = drv_data->value [chan-SENSOR_CHAN_ACCEL_X];
        val->val1 = 0; // calibration adjusted
        val->val2 = value; // raw value
        if (value >= drv_cfg->min_on) {
            val->val1 =  1 + value - drv_cfg->min_on;
        } else if (value <= -drv_cfg->min_on) {
            val->val1 = -1 + value + drv_cfg->min_on;
        }
        return 0;
    }
    
    return -ENOTSUP;
}

static void zmk_joy_work(struct k_work *work) {
    struct joy_data *drv_data = CONTAINER_OF(work, struct joy_data, work);
    const struct joy_config *drv_cfg = drv_data->dev->config;
    if (drv_data->setup) {
        k_timer_stop(&drv_data->timer);

        joy_get_state (drv_data->dev);

        bool on = false;

        for (int axis=0; axis<drv_data->num_axis; axis++) {
            if (abs(drv_data->value [axis]) >= drv_cfg->min_on) {
                drv_data->handler (drv_data->dev, drv_data->trigger);
                on = true;
                LOG_DBG("Joystick triggered: axis=%d, val=%d, on=%d", axis, drv_data->value [axis], drv_data->on);
                break; // only need to trigger once, not per-axis
            }
        }
        if (drv_data->on && !on) {
            // joystick was 'on', so now need to tell handler it's off to release any keys
            drv_data->handler (drv_data->dev, drv_data->trigger);
            LOG_DBG("Joystick triggered (off)");
        }
        drv_data->on = on;
        // reset timer because of the delay so it doesn't overwhelm the system with timers
        k_timer_start(&drv_data->timer, K_MSEC(1000/drv_cfg->frequency), K_MSEC(1000/drv_cfg->frequency));
    }
}

static void zmk_joy_timer(struct k_timer *timer) { 
    struct joy_data *drv_data = CONTAINER_OF(timer, struct joy_data, timer);
    k_work_submit(&drv_data->work); 
}

int joy_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                    sensor_trigger_handler_t handler) {
    struct joy_data *drv_data = dev->data;
    const struct joy_config *drv_cfg = dev->config;

    k_timer_stop (&drv_data->timer);
    
    drv_data->trigger = trig;
    drv_data->handler = handler;
    drv_data->on = false;
    
    k_work_init (&drv_data->work, zmk_joy_work);
    k_timer_init (&drv_data->timer, zmk_joy_timer, NULL);
    k_timer_user_data_set (&drv_data->timer, (void*)dev);
    k_timer_start(&drv_data->timer, K_MSEC(1000/drv_cfg->frequency), K_MSEC(1000/drv_cfg->frequency));
    
    return 0;
}


static const struct sensor_driver_api joy_driver_api = {
    .trigger_set = joy_trigger_set,
    .sample_fetch = joy_sample_fetch,
    .channel_get = joy_channel_get,
};


int joy_init(const struct device *dev) {
    struct joy_data *drv_data = dev->data;
    const struct joy_config *drv_cfg = dev->config;

    drv_data->dev = dev;
    drv_data->setup = false;
    drv_data->num_axis = drv_cfg->num_axis;
    
    for (int axis=0; axis<drv_data->num_axis; axis++) {
        drv_data->adc [axis] = drv_cfg->adc [axis];
        if (drv_data->adc [axis] == NULL) {
            LOG_ERR("Joy: Failed to get pointer to ADC device %d", axis);
            return -EINVAL;
        }
    
        drv_data->as [axis] = (struct adc_sequence){
            .channels = BIT(drv_cfg->io_channel [axis]+1), /* Has to be channel +1 because channel 0 is used for the battery */
            .buffer = &(drv_data->adc_raw [axis]),
            .buffer_size = sizeof(drv_data->adc_raw [axis]),
            .oversampling = 4,
            .calibrate = true,
            .resolution = 12
        };

#ifdef CONFIG_ADC_NRFX_SAADC
        drv_data->acc [axis] = (struct adc_channel_cfg){
            .gain = ADC_GAIN_1_4,
            .reference = ADC_REF_VDD_1_4,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + drv_cfg->io_channel [axis],
            .channel_id = drv_cfg->io_channel [axis]+1
        };
#else
#error Unsupported ADC
#endif

        int rc = adc_channel_setup(drv_data->adc [axis], &(drv_data->acc [axis]));
        LOG_DBG("Joy AIN%u setup returned %d", drv_cfg->io_channel [axis], rc);

    }
    
    ext_power = device_get_binding("EXT_POWER");
    if (ext_power == NULL) {
        LOG_ERR("Unable to retrieve ext_power device: EXT_POWER");
    }
    
    drv_data->setup = true;

    joy_get_state(dev);
    
    for (int axis=0; axis<drv_data->num_axis; axis++) {
        drv_data->zero_value [axis] = drv_data->value [axis];
        drv_data->delta [axis] = 0;
        drv_data->last_rotate [axis] = 0;
    }
    
    return 0;
}

#define JOY_INST(n)                                                                              \
    struct joy_data joy_data_##n;                                                                \
    const struct joy_config joy_cfg_##n = {                                                      \
        .io_channel [0] = DT_INST_IO_CHANNELS_INPUT_BY_IDX(n, 0),				 \
        .io_channel [1] = DT_INST_IO_CHANNELS_INPUT_BY_IDX(n, 1),				 \
        .num_axis = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, num_axis), (2), (DT_INST_PROP(n, num_axis))),										 \
        .adc [0] = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR_BY_IDX(n,0)),                           \
        .adc [1] = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR_BY_IDX(n,1)),                           \
        .resolution = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, resolution), (1), (DT_INST_PROP(n, resolution))),   \
        .min_on = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, min_on), (1), (DT_INST_PROP(n, min_on))),           \
        .frequency = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, frequency), (1), (DT_INST_PROP(n, frequency))),     \
        .reverse = COND_CODE_0(DT_INST_NODE_HAS_PROP(n, reverse), (1), (DT_INST_PROP(n, reverse))),         \
    };                                                                                           \
    DEVICE_DT_INST_DEFINE(n, joy_init, device_pm_control_nop, &joy_data_##n, &joy_cfg_##n,       \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &joy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(JOY_INST)

