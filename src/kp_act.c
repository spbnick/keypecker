/** @file
 *  @brief Keypecker actuator
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_act.h"
#include <zephyr/kernel.h>

/*
 * Only changed upon initialization.
 */

/** The GPIO port device */
static const struct device * volatile kp_act_gpio = NULL;

/** The "disable" output GPIO pin */
static gpio_pin_t kp_act_gpio_pin_disable;

/** The "dir" output GPIO pin */
static gpio_pin_t kp_act_gpio_pin_dir;

/** The "step" output GPIO pin */
static gpio_pin_t kp_act_gpio_pin_step;

/*
 * General state changed by all functions
 */
/** The mutex protecting all the state */
static K_MUTEX_DEFINE(kp_act_mutex);

/** Execute the following statement with state mutex held */
#define KP_ACT_WITH_MUTEX \
	for (int _i = (k_mutex_lock(&kp_act_mutex, K_FOREVER), 0); \
	     _i == 0; \
	     _i = (k_mutex_unlock(&kp_act_mutex), 1))

/** Last power */
static kp_act_power kp_act_power_last;

/** Current power */
static kp_act_power kp_act_power_curr;

/** The current actuator position, in steps */
static int32_t kp_act_pos_steps;

/** True if a move has to be aborted, false otherwise */
static volatile bool kp_act_move_aborted;

/*
 * Movement state accessed by move.../locate functions
 */
/** The additional mutex protecting the movement state */
static K_MUTEX_DEFINE(kp_act_move_mutex);

/** Execute the following statement with movement mutex held */
#define KP_ACT_WITH_MOVE_MUTEX \
	for (int _i = (k_mutex_lock(&kp_act_move_mutex, K_FOREVER), 0); \
	     _i == 0; \
	     _i = (k_mutex_unlock(&kp_act_move_mutex), 1))

/** The move target position, in steps */
static int32_t kp_act_target_steps;

/*
 * End of state
 */

kp_act_power
kp_act_on(void)
{
	/*
	 * If the power is off, we should always be able to turn it on.
	 */
	kp_act_power power = KP_ACT_POWER_INVALID;
	assert(kp_act_is_initialized());
	KP_ACT_WITH_MUTEX {
		if (kp_act_power_is_on(kp_act_power_curr)) {
			continue;
		}
		gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_disable, 0);
		if (kp_act_power_last >= KP_ACT_POWER_ON_MAX) {
			power = KP_ACT_POWER_ON_MIN;
		} else {
			power = kp_act_power_last + 1;
		}
		kp_act_power_curr = power;
	}
	return power;
}

bool
kp_act_off(kp_act_power power)
{
	/*
	 * If the correct power-on is supplied, we should always be able to
	 * turn off the power, regardless whether we're moving or not.
	 * If we're moving, the move aborts with KP_ACT_MOVE_RC_OFF.
	 */
	bool turned_off = false;
	assert(kp_act_power_is_valid(power));
	assert(kp_act_is_initialized());
	if (kp_act_power_is_on(power)) {
		KP_ACT_WITH_MUTEX {
			if (power != kp_act_power_curr) {
				continue;
			}
			gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_disable, 1);
			kp_act_power_last = kp_act_power_curr;
			kp_act_power_curr = KP_ACT_POWER_OFF;
			kp_act_pos_steps = 0;
			turned_off = true;
		}
	}
	return turned_off;
}

kp_act_pos
kp_act_locate(void)
{
	kp_act_pos pos;
	assert(kp_act_is_initialized());
	/*
	 * We should always be able to return the location, regardless of the
	 * power status, but location would be invalid if the power is off.
	 */
	KP_ACT_WITH_MUTEX {
		pos = kp_act_pos_create(kp_act_power_curr, kp_act_pos_steps);
	}
	return pos;
}

/** Move timer */
static K_TIMER_DEFINE(kp_act_move_timer, NULL, NULL);

/** Move timer period */
#define KP_ACT_MOVE_TIMER_PERIOD	K_MSEC(1)

/**
 * Wait for next timer tick, or go to a label if timer is stopped.
 *
 * @param _stop_label	The label to go to if timer is stopped.
 */
#define KP_ACT_MOVE_TIMER_SYNC(_stop_label) \
	do {                                                    \
		/* If the timer is stopped or not started */    \
		if (!k_timer_status_sync(&kp_act_move_timer)) { \
			goto _stop_label;                       \
		}                                               \
	} while (0)

enum kp_act_move_rc
kp_act_move(kp_act_power power, bool relative, int32_t steps)
{
	enum kp_act_move_rc rc;
	assert(kp_act_power_is_valid(power));
	assert(kp_act_is_initialized());
	KP_ACT_WITH_MOVE_MUTEX {
		/* Start the timer */
		KP_ACT_WITH_MUTEX {
			/* If we have the current power on */
			if (!kp_act_power_is_on(power) ||
			    power != kp_act_power_curr) {
				rc = KP_ACT_MOVE_RC_OFF;
				continue;
			}
			rc = KP_ACT_MOVE_RC_OK;
			/* Calculate target steps */
			kp_act_target_steps =
				relative ? kp_act_pos_steps + steps : steps;
			/* If we don't have to move */
			if (kp_act_target_steps == kp_act_pos_steps) {
				continue;
			}
			kp_act_move_aborted = false;
			k_timer_start(&kp_act_move_timer,
					KP_ACT_MOVE_TIMER_PERIOD,
					KP_ACT_MOVE_TIMER_PERIOD);
		}

		/* Run the timer */
		while (true) {
			/* Control */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			KP_ACT_WITH_MUTEX {
				if (!kp_act_power_is_on(kp_act_power_curr)) {
					rc = KP_ACT_MOVE_RC_OFF;
					k_timer_stop(&kp_act_move_timer);
					continue;
				}
				if (kp_act_move_aborted) {
					rc = KP_ACT_MOVE_RC_ABORTED;
					k_timer_stop(&kp_act_move_timer);
					continue;
				}
				if (kp_act_target_steps == kp_act_pos_steps) {
					rc = KP_ACT_MOVE_RC_OK;
					k_timer_stop(&kp_act_move_timer);
					continue;
				}
				gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_dir,
					     kp_act_pos_steps > kp_act_target_steps);
			}
			/* Raise */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_step, 1);
			/* Hold */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			KP_ACT_WITH_MUTEX {
				if (kp_act_pos_steps < kp_act_target_steps) {
					kp_act_pos_steps++;
				} else {
					kp_act_pos_steps--;
				}
			}
			/* Fall */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_step, 0);
		}
stop:;
	}

	return rc;
}

#undef KP_ACT_MOVE_TIMER_SYNC


bool
kp_act_abort(kp_act_power power)
{
	bool aborted = false;
	assert(kp_act_power_is_valid(power));
	assert(kp_act_is_initialized());
	KP_ACT_WITH_MUTEX {
		if (!kp_act_power_is_on(power) ||
		    power != kp_act_power_curr) {
			continue;
		}
		kp_act_move_aborted = true;
		aborted = true;
	}
	return aborted;
}

bool
kp_act_is_initialized(void)
{
	return kp_act_gpio != NULL;
}

void
kp_act_init(const struct device *gpio,
		gpio_pin_t disable_pin,
		gpio_pin_t dir_pin,
		gpio_pin_t step_pin)
{
	assert(gpio != NULL);
	assert(device_is_ready(gpio));
	assert(!kp_act_is_initialized());

	/* Move state */
	kp_act_target_steps = 0;

	/* General state */
	kp_act_power_last = KP_ACT_POWER_OFF;
	kp_act_power_curr = KP_ACT_POWER_OFF;
	kp_act_pos_steps = 0;
	kp_act_move_aborted = false;

	/* Init state */
	kp_act_gpio_pin_disable = disable_pin;
	kp_act_gpio_pin_dir = dir_pin;
	kp_act_gpio_pin_step = step_pin;
	gpio_pin_configure(gpio, kp_act_gpio_pin_disable,
				GPIO_OPEN_DRAIN | GPIO_OUTPUT_HIGH);
	gpio_pin_configure(gpio, kp_act_gpio_pin_dir,
				GPIO_PUSH_PULL | GPIO_OUTPUT_LOW);
	gpio_pin_configure(gpio, kp_act_gpio_pin_step,
				GPIO_PUSH_PULL | GPIO_OUTPUT_LOW);

	/* Mark actuator initialized */
	kp_act_gpio = gpio;
}
