/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>

#ifndef JOY_NUM_AXIS
#define JOY_NUM_AXIS 3
#endif

struct io_channel_config {
    const char *label;
    uint8_t channel;
};

struct joy_config {
    int io_channel [JOY_NUM_AXIS];
    const struct device *(adc [JOY_NUM_AXIS]);
    int resolution;
    int min_on;
    int frequency;
    bool reverse;
    uint8_t num_axis;
};

struct joy_data {
    const struct device *(adc [JOY_NUM_AXIS]);
    bool setup;
    bool on;
    uint8_t num_axis;
    
    struct adc_channel_cfg acc [JOY_NUM_AXIS];
    struct adc_sequence as [JOY_NUM_AXIS];
    uint16_t adc_raw [JOY_NUM_AXIS];

    int zero_value [JOY_NUM_AXIS];
    int value [JOY_NUM_AXIS];
    int delta [JOY_NUM_AXIS];
    int last_rotate [JOY_NUM_AXIS];

    const struct device *dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;

    struct k_timer timer;
    struct k_work work;

};
