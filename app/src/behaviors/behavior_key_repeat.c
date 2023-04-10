/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_key_repeat

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/hid.h>

#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ZMK_BHV_KEY_REPEAT_MAX_IGNORES 10

struct behavior_key_repeat_config {
    uint8_t index;
    uint16_t ignore_keycodes_count;
    uint32_t ignore_keycodes[ZMK_BHV_KEY_REPEAT_MAX_IGNORES];
    uint8_t usage_pages_count;
    // does it make more sense for ignore keycodes be variable length and usage pages be fixed?
    uint16_t usage_pages[];
};

struct behavior_key_repeat_data {
    struct zmk_keycode_state_changed last_keycode_pressed;
    struct zmk_keycode_state_changed current_keycode_pressed;
};

static int on_key_repeat_binding_pressed(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    struct behavior_key_repeat_data *data = dev->data;

    if (data->last_keycode_pressed.usage_page == 0) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    memcpy(&data->current_keycode_pressed, &data->last_keycode_pressed,
           sizeof(struct zmk_keycode_state_changed));
    data->current_keycode_pressed.timestamp = k_uptime_get();

    ZMK_EVENT_RAISE(new_zmk_keycode_state_changed(data->current_keycode_pressed));

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_key_repeat_binding_released(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    struct behavior_key_repeat_data *data = dev->data;

    if (data->current_keycode_pressed.usage_page == 0) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    data->current_keycode_pressed.timestamp = k_uptime_get();
    data->current_keycode_pressed.state = false;

    ZMK_EVENT_RAISE(new_zmk_keycode_state_changed(data->current_keycode_pressed));
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_key_repeat_driver_api = {
    .binding_pressed = on_key_repeat_binding_pressed,
    .binding_released = on_key_repeat_binding_released,
};

static int key_repeat_keycode_state_changed_listener(const zmk_event_t *eh);

ZMK_LISTENER(behavior_key_repeat, key_repeat_keycode_state_changed_listener);
ZMK_SUBSCRIPTION(behavior_key_repeat, zmk_keycode_state_changed);

static const struct device *devs[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];

static int key_repeat_keycode_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    for (int i = 0; i < DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT); i++) {
        const struct device *dev = devs[i];
        if (dev == NULL) {
            continue;
        }

        struct behavior_key_repeat_data *data = dev->data;
        const struct behavior_key_repeat_config *config = dev->config;

        bool is_ignored_keycode = false;
        for (int u = 0; u < config->ignore_keycodes_count; u++) {
            LOG_DBG("checking ignored keycode %X with pressed keycode %X",
                    config->ignore_keycodes[u], ev->keycode);
            if (config->ignore_keycodes[u] == ZMK_HID_USAGE(ev->usage_page, ev->keycode)) {
                is_ignored_keycode = true;
                break;
            }
        }
        if (!is_ignored_keycode) {
            for (int u = 0; u < config->usage_pages_count; u++) {
                if (config->usage_pages[u] == ev->usage_page) {
                    memcpy(&data->last_keycode_pressed, ev,
                           sizeof(struct zmk_keycode_state_changed));
                    data->last_keycode_pressed.implicit_modifiers |= zmk_hid_get_explicit_mods();
                    break;
                }
            }
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

static int behavior_key_repeat_init(const struct device *dev) {
    const struct behavior_key_repeat_config *config = dev->config;
    devs[config->index] = dev;
    return 0;
}

#define KR_INST(n)                                                                                 \
    static struct behavior_key_repeat_data behavior_key_repeat_data_##n = {};                      \
    static struct behavior_key_repeat_config behavior_key_repeat_config_##n = {                    \
        .index = n,                                                                                \
        .ignore_keycodes = DT_INST_PROP(n, ignore_keycodes),                                       \
        .ignore_keycodes_count = DT_INST_PROP_LEN(n, ignore_keycodes),                             \
        .usage_pages = DT_INST_PROP(n, usage_pages),                                               \
        .usage_pages_count = DT_INST_PROP_LEN(n, usage_pages),                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, behavior_key_repeat_init, NULL, &behavior_key_repeat_data_##n,        \
                          &behavior_key_repeat_config_##n, APPLICATION,                            \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_key_repeat_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KR_INST)

#endif
