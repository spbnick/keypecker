/** @file
 *  @brief Keypecker actuator
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_act.h"

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
/** The spinlock protecting the base state */
static struct k_spinlock kp_act_lock = {};

/** Execute the following statement with base state lock held */
#define KP_ACT_WITH_LOCK \
	for (k_spinlock_key_t _key = k_spin_lock(&kp_act_lock), \
			      _i = {0}; \
	     _i.key == 0; \
	     k_spin_unlock(&kp_act_lock, _key), _i.key = 1)

/** The current actuator position, in steps */
static volatile int32_t kp_act_pos;

/** True if a move has to be aborted, false otherwise */
static volatile bool kp_act_move_aborted;

/*
 * Movement state accessed by move.../locate functions
 */

/** The semaphore signaling a movement can be started */
static K_SEM_DEFINE(kp_act_move_available, 1, 1);

/** The semaphore signaling a movement should begin */
static K_SEM_DEFINE(kp_act_move_begin, 0, 1);

/** The move target position, in steps */
static int32_t kp_act_target;

/** The timestamp of the last step in cycles */
static uint32_t kp_act_move_last_cycles;

/** True if the last step was in the positive direction, false otherwise */
static bool kp_act_move_last_positive;

/** The semaphore signaling a movement is done */
static K_SEM_DEFINE(kp_act_move_done, 0, 1);

/** The move result, only valid when kp_act_move_done is available */
static volatile enum kp_act_move_rc kp_act_move_rc;

/*
 * End of state
 */

/**
 * Check if the actuator power is off, assuming the state mutex is held.
 *
 * @return True if the power is off, false if on.
 */
static inline bool
kp_act_is_off_locked(void)
{
	return gpio_pin_get(kp_act_gpio, kp_act_gpio_pin_disable);
}

/**
 * Check if the actuator power is on, assuming the state mutex is held.
 *
 * @return True if the power is on, false if off.
 */
static inline bool
kp_act_is_on_locked(void)
{
	return !kp_act_is_off_locked();
}

bool
kp_act_is_off(void)
{
	bool is_off;
	assert(kp_act_is_initialized());
	KP_ACT_WITH_LOCK {
		is_off = kp_act_is_off_locked();
	}
	return is_off;
}

bool
kp_act_on(void)
{
	bool turned_on = false;
	assert(kp_act_is_initialized());
	KP_ACT_WITH_LOCK {
		if (kp_act_is_off_locked()) {
			gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_disable, 0);
			turned_on = true;
		}
	}
	return turned_on;
}

bool
kp_act_off(void)
{
	bool turned_off = false;
	assert(kp_act_is_initialized());
	KP_ACT_WITH_LOCK {
		/* If we're moving, the move aborts with KP_ACT_MOVE_RC_OFF */
		if (kp_act_is_on_locked()) {
			gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_disable, 1);
			kp_act_pos = 0;
			turned_off = true;
		}
	}
	return turned_off;
}

int32_t
kp_act_locate(void)
{
	int32_t pos;
	assert(kp_act_is_initialized());
	KP_ACT_WITH_LOCK {
		pos = kp_act_is_on_locked()
			? kp_act_pos
			: KP_ACT_POS_INVALID;
	}
	return pos;
}

/** Move timer */
static K_TIMER_DEFINE(kp_act_move_timer, NULL, NULL);

/** Minimum move timer period, us */
#define KP_ACT_MOVE_TIMER_PERIOD_MIN_US	400

/** Maximum move timer period, us */
#define KP_ACT_MOVE_TIMER_PERIOD_MAX_US	4000

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

void
kp_act_move_thread_fn(void *arg1, void *arg2, void *arg3)
{
	bool positive = false;
	assert(kp_act_is_initialized());

	/* While we can get the "begin" semaphore */
	while (k_sem_take(&kp_act_move_begin, K_FOREVER) == 0) {
		/* Run the timer */
		while (true) {
			/* Control */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			KP_ACT_WITH_LOCK {
				if (kp_act_is_off_locked()) {
					kp_act_move_rc = KP_ACT_MOVE_RC_OFF;
					k_timer_stop(&kp_act_move_timer);
					continue;
				}
				if (kp_act_move_aborted) {
					kp_act_move_rc =
						KP_ACT_MOVE_RC_ABORTED;
					k_timer_stop(&kp_act_move_timer);
					continue;
				}
				if (kp_act_target == kp_act_pos) {
					kp_act_move_rc = KP_ACT_MOVE_RC_OK;
					k_timer_stop(&kp_act_move_timer);
					continue;
				}
				positive = kp_act_target > kp_act_pos;
				gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_dir,
					     !positive);
			}
			/* Raise */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_step, 1);
			kp_act_move_last_cycles = k_cycle_get_32();
			kp_act_move_last_positive = positive;
			/* Hold */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			KP_ACT_WITH_LOCK {
				if (positive) {
					kp_act_pos++;
				} else {
					kp_act_pos--;
				}
			}
			/* Fall */
			KP_ACT_MOVE_TIMER_SYNC(stop);
			gpio_pin_set(kp_act_gpio, kp_act_gpio_pin_step, 0);
		}
stop:;
		k_sem_give(&kp_act_move_done);
	}
}

#undef KP_ACT_MOVE_TIMER_SYNC

/** The thread moving the actuator */
K_THREAD_DEFINE(kp_act_move_thread, 512,
		kp_act_move_thread_fn, NULL, NULL, NULL,
		-1, 0, -1);

void
kp_act_start_move(bool relative, int32_t steps, uint32_t speed)
{
	bool started = false;
	uint32_t period_us;
	k_timeout_t delay = K_NO_WAIT;
	assert(relative || kp_act_pos_is_valid(steps));
	assert(speed <= 100);
	assert(kp_act_is_initialized());

	/* Wait for a move to be available */
	k_sem_take(&kp_act_move_available, K_FOREVER);

	/* Start the timer */
	KP_ACT_WITH_LOCK {
		/* If we have the power off */
		if (kp_act_is_off_locked()) {
			kp_act_move_rc = KP_ACT_MOVE_RC_OFF;
			continue;
		}
		/* Calculate target steps */
		kp_act_target =
			relative ? kp_act_pos + steps : steps;
		/* If we don't have to move */
		if (kp_act_target == kp_act_pos) {
			kp_act_move_rc = KP_ACT_MOVE_RC_OK;
			continue;
		}

		kp_act_move_aborted = false;

		/* If our direction is different from the last step */
		if ((kp_act_target > kp_act_pos) !=
				kp_act_move_last_positive) {
			/*
			 * Minimum turn delay, microseconds.
			 * The faster we move, the more time we need to absorb
			 * the momentum.
			 */
			uint32_t turn_delay_us =
				(KP_ACT_MOVE_TIMER_PERIOD_MIN_US +
				 ((KP_ACT_MOVE_TIMER_PERIOD_MAX_US -
				   KP_ACT_MOVE_TIMER_PERIOD_MIN_US) *
				  speed) / 100) * 2;
			/* Current time, cycles */
			uint32_t now_cycles = k_cycle_get_32();
			/* Microseconds from the last step */
			uint32_t elapsed_us = k_cyc_to_us_floor32(
				now_cycles > kp_act_move_last_cycles
					? now_cycles - kp_act_move_last_cycles
					/* Just always wait on overflow */
					: 0
			);
			if (elapsed_us < turn_delay_us) {
				/* Wait to absorb momentum before turning */
				delay = K_USEC(turn_delay_us - elapsed_us);
			}
		}

		/* Shorter period for faster movement */
		period_us = KP_ACT_MOVE_TIMER_PERIOD_MAX_US -
			((KP_ACT_MOVE_TIMER_PERIOD_MAX_US -
			  KP_ACT_MOVE_TIMER_PERIOD_MIN_US) *
			 speed) / 100;

		k_timer_start(&kp_act_move_timer, delay, K_USEC(period_us));
		started = true;
	}

	if (started) {
		/* Begin the move */
		k_sem_give(&kp_act_move_begin);
	} else {
		/* Complete the move */
		k_sem_give(&kp_act_move_done);
	}
}

void
kp_act_finish_move_event_init(struct k_poll_event *event)
{
	assert(kp_act_is_initialized());
	k_poll_event_init(event, K_POLL_TYPE_SEM_AVAILABLE,
			  K_POLL_MODE_NOTIFY_ONLY, &kp_act_move_done);
}

enum kp_act_move_rc
kp_act_finish_move(k_timeout_t timeout)
{
	enum kp_act_move_rc rc;
	assert(kp_act_is_initialized());

	/* Wait for the move to finish */
	if (k_sem_take(&kp_act_move_done, timeout)) {
		return KP_ACT_MOVE_TIMEOUT;
	}

	/* Remember the result */
	rc = kp_act_move_rc;

	/* Mark next move as available */
	k_sem_give(&kp_act_move_available);

	/* Return the remembered result */
	return rc;
}

bool
kp_act_abort(void)
{
	bool aborted = false;
	assert(kp_act_is_initialized());
	KP_ACT_WITH_LOCK {
		if (kp_act_is_off_locked()) {
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
	kp_act_target = 0;
	kp_act_move_last_cycles = 0;
	kp_act_move_last_positive = false;

	/* General state */
	kp_act_pos = 0;
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

	/* Start the actuator-moving thread */
	k_thread_start(kp_act_move_thread);

	assert(kp_act_is_initialized());
}
