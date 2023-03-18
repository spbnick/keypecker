/** @file
 *  @brief Keypecker actuator
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_ACT_H_
#define KP_ACT_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <stdbool.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the actuator to a powered-off state.
 *
 * @param gpio		The device for the GPIO port to use to control the
 *			actuator.
 * @param disable_pin	The disable pin number on the GPIO port.
 * @param dir_pin	The dir pin number on the GPIO port.
 * @param step_pin	The step pin number on the GPIO port.
 */
extern void kp_act_init(const struct device *gpio,
			gpio_pin_t disable_pin,
			gpio_pin_t dir_pin,
			gpio_pin_t step_pin);

/**
 * Check if the actuator is initialized.
 *
 * @return True if the actuator is initialized, false if not.
 */
extern bool kp_act_is_initialized(void);

/**
 * Check if the actuator power is off.
 *
 * @return True if the power is off, false if on.
 */
extern bool kp_act_is_off(void);

/**
 * Check if the actuator power is on.
 *
 * @return True if the power is on, false if off.
 */
static inline bool
kp_act_is_on(void)
{
	assert(kp_act_is_initialized());
	return !kp_act_is_off();
}

/**
 * Turn the actuator power on, if not on already.
 *
 * @return True if the power was turned on, false if it was already on.
 */
extern bool kp_act_on(void);

/**
 * Turn the actuator power off.
 *
 * @return True if the power turned off, false if it was already off.
 */
extern bool kp_act_off(void);

/** Invalid (unavailable) actuator position */
#define KP_ACT_POS_INVALID	INT32_MIN

/** Minimum possible actuator position */
#define KP_ACT_POS_MIN		(INT32_MIN + 1)

/** Maximum possible actuator position */
#define KP_ACT_POS_MAX		INT32_MAX

/**
 * Check if a position is valid.
 *
 * @param pos	The position to check.
 *
 * @return True if the position is valid, false otherwise.
 */
static inline bool kp_act_pos_is_valid(int32_t pos)
{
	return pos != KP_ACT_POS_INVALID;
}

/**
 * Retrieve the absolute position of a powered actuator.
 *
 * @return The position, if successful, or KP_ACT_POS_INVALID, if the actuator
 * 	   is powered off.
 */
extern int32_t kp_act_locate(void);

/** Result code of a movement attempt */
enum kp_act_move_rc {
	/** Move succeeded / finished */
	KP_ACT_MOVE_RC_OK,
	/** Move is aborted */
	KP_ACT_MOVE_RC_ABORTED,
	/** Actuator is off (power is invalid) */
	KP_ACT_MOVE_RC_OFF,
	/** Waiting for a move to finish timed out */
	KP_ACT_MOVE_TIMEOUT,
};

/**
 * Start moving the actuator.
 *
 * @param relative	True if the move is relative, false if absolute.
 * @param steps		The number of steps to move by, if "relative" is true.
 * 			The absolute position, in steps, if "relative" is
 * 			false. Positive - lower, negative - higher.
 * @param speed		The speed with which to move, 0-100%.
 */
extern void kp_act_start_move(bool relative, int32_t steps, uint32_t speed);

/**
 * Finish moving the actuator.
 *
 * @param timeout	The time to wait for the move to finish, or one of
 * 			the special values K_NO_WAIT and K_FOREVER.
 *
 * @return The move result code.
 */
extern enum kp_act_move_rc kp_act_finish_move(k_timeout_t timeout);

/**
 * Initialize a poll event to wait for finished actuator moves.
 *
 * @param event	The poll event to initialize.
 */
extern void kp_act_finish_move_event_init(struct k_poll_event *event);

/**
 * Move the actuator.
 *
 * @param relative	True if the move is relative, false if absolute.
 * @param steps		The number of steps to move by, if "relative" is true.
 * 			The absolute position, in steps, if "relative" is
 * 			false. Positive - lower, negative - higher.
 * @param speed		The speed with which to move, 0-100%.
 *
 * @return The move result code.
 */
static inline enum kp_act_move_rc kp_act_move(bool relative,
					      int32_t steps,
					      uint32_t speed)
{
	assert(relative || kp_act_pos_is_valid(steps));
	assert(speed <= 100);
	assert(kp_act_is_initialized());
	kp_act_start_move(relative, steps, speed);
	return kp_act_finish_move(K_FOREVER);
}

/**
 * Move the actuator to an absolute position.
 * Waits for the previous move to be finished/aborted.
 *
 * @param pos	The absolute position to move the actuator to (must be valid).
 * @param speed	The speed with which to move, 0-100%.
 *
 * @return The movement result.
 */
static inline enum kp_act_move_rc
kp_act_move_to(int32_t pos, uint32_t speed)
{
	assert(kp_act_pos_is_valid(pos));
	assert(speed <= 100);
	assert(kp_act_is_initialized());
	return kp_act_move(false, pos, speed);
}

/**
 * Start moving the actuator to an absolute position.
 * Waits for the previous move to be finished/aborted.
 *
 * @param pos	The absolute position to move the actuator to (must be valid).
 * @param speed	The speed with which to move, 0-100%.
 */
static inline void
kp_act_start_move_to(int32_t pos, uint32_t speed)
{
	assert(kp_act_pos_is_valid(pos));
	assert(speed <= 100);
	assert(kp_act_is_initialized());
	kp_act_start_move(false, pos, speed);
}

/**
 * Move the actuator by a specified number of steps.
 *
 * @param steps	The number of steps to move by: positive values
 * 		- lower, negative - higher.
 * @param speed	The speed with which to move, 0-100%.
 *
 * @return The movement result.
 */
static inline enum kp_act_move_rc
kp_act_move_by(int32_t steps, uint32_t speed)
{
	assert(speed <= 100);
	assert(kp_act_is_initialized());
	return kp_act_move(true, steps, speed);
}

/**
 * Start moving the actuator by a specified number of steps.
 *
 * @param steps	The number of steps to move by: positive values
 * 		- lower, negative - higher.
 * @param speed	The speed with which to move, 0-100%.
 *
 * @return The movement result.
 */
static inline void
kp_act_start_move_by(int32_t steps, uint32_t speed)
{
	assert(speed <= 100);
	assert(kp_act_is_initialized());
	kp_act_start_move(true, steps, speed);
}

/**
 * Abort the actuator's movement in progress, if any.
 *
 * @return True if there was no movement or it was aborted,
 * 	   false if the actuator was not powered.
 */
extern bool kp_act_abort(void);

#ifdef __cplusplus
}
#endif

#endif /* KP_ACT_H_ */
