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

/** Actuator power */
typedef uint8_t kp_act_power;

/** Power off */
#define KP_ACT_POWER_OFF	(kp_act_power)0

/** Minimum valid power-on */
#define KP_ACT_POWER_ON_MIN	(kp_act_power)1

/** Maximum valid power-on */
#define KP_ACT_POWER_ON_MAX	(kp_act_power)(UINT8_MAX - 1)

/** Invalid (unavailable) power */
#define KP_ACT_POWER_INVALID	(kp_act_power)UINT8_MAX

/**
 * Check if the specified power is valid.
 *
 * @param power	The power to check.
 *
 * @return True if the specified power is valid, false if off.
 */
static inline bool
kp_act_power_is_valid(kp_act_power power)
{
	return power != KP_ACT_POWER_INVALID;
}

/**
 * Check if the specified power is on.
 *
 * @param power	The power to check.
 *
 * @return True if the specified power is on, false if off.
 */
static inline bool
kp_act_power_is_on(kp_act_power power)
{
	return power >= KP_ACT_POWER_ON_MIN &&
		power <= KP_ACT_POWER_ON_MAX;
}

/**
 * Turn the actuator power on, if not on already.
 *
 * @return A valid power-on, if successful, or KP_ACT_POWER_INVALID, if the
 * 		power was already on.
 */
extern kp_act_power kp_act_on(void);

/**
 * Turn the actuator power off.
 *
 * @param power	The power to turn off (must be valid).
 *
 * @return True if the power was on, and was turned off,
 * 	   false if the power was already off.
 */
extern bool kp_act_off(kp_act_power power);

/** Position of a powered actuator ((steps << 8) | power) */
typedef int32_t kp_act_pos;

/** Invalid (unavailable) actuator position */
#define KP_ACT_POS_INVALID	(kp_act_pos)0

/**
 * Check if a position is valid.
 *
 * @param pos	The position to check.
 *
 * @return True if the position is valid, false otherwise.
 */
static inline bool kp_act_pos_is_valid(kp_act_pos pos)
{
	return pos != KP_ACT_POS_INVALID;
}

/**
 * Create a powered position from a power and steps.
 *
 * @param power	The power to create position for (must be valid).
 * @param steps	The position steps to use.
 *
 * @return The created powered position.
 */
static inline kp_act_pos
kp_act_pos_create(kp_act_power power, int32_t steps)
{
	assert(kp_act_power_is_valid(power));
	if (kp_act_power_is_on(power)) {
		return (steps << 8) | power;
	} else {
		return KP_ACT_POS_INVALID;
	}
}

/**
 * Retrieve the power of a position.
 *
 * @param pos	The position to retrieve the power from (must be valid).
 *
 * @return The retrieved power, or KP_ACT_POWER_OFF, if the power is off.
 */
static inline kp_act_power
kp_act_pos_get_power(kp_act_pos pos)
{
	assert(kp_act_pos_is_valid(pos));
	return pos & 0xff;
}

/**
 * Retrieve the absolute steps of a position.
 *
 * @param pos	The position to retrieve the steps from.
 *
 * @return The retrieved steps.
 */
static inline int32_t
kp_act_pos_get_steps(kp_act_pos pos)
{
	return pos >> 8;
}

/**
 * Retrieve the absolute position of a powered actuator.
 *
 * @return The position, if successful, or KP_ACT_POS_INVALID, if the actuator
 * 	   is powered off.
 */
extern kp_act_pos kp_act_locate(void);

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
 * @param power		The power with which the actuator is moved (must be
 *			valid).
 * @param relative	True if the move is relative, false if absolute.
 * @param steps		The number of steps to move by, if "relative" is true
 * 			(positive - lower, negative - higher),
 * 			the absolute position within the power, in steps, if
 * 			"relative" is false.
 */
extern void kp_act_start_move(kp_act_power power,
			      bool relative, int32_t steps);

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
 * @param power		The power with which the actuator is moved (must be
 *			valid).
 * @param relative	True if the move is relative, false if absolute.
 * @param steps		The number of steps to move by, if "relative" is true
 * 			(positive - lower, negative - higher),
 * 			the absolute position within the power, in steps, if
 * 			"relative" is false.
 *
 * @return The move result code.
 */
static inline enum kp_act_move_rc kp_act_move(kp_act_power power,
					      bool relative, int32_t steps)
{
	assert(kp_act_power_is_valid(power));
	assert(kp_act_is_initialized());
	kp_act_start_move(power, relative, steps);
	return kp_act_finish_move(K_FOREVER);
}

/**
 * Move the actuator to a powered absolute position.
 * Waits for the previous move to be finished/aborted.
 *
 * @param pos	The powered absolute position to move the actuator to (must be
 *		valid).
 *
 * @return The movement result.
 */
static inline enum kp_act_move_rc
kp_act_move_to(kp_act_pos pos)
{
	assert(kp_act_pos_is_valid(pos));
	assert(kp_act_is_initialized());
	return kp_act_move(kp_act_pos_get_power(pos),
			   false, kp_act_pos_get_steps(pos));
}

/**
 * Start moving the actuator to a powered absolute position.
 * Waits for the previous move to be finished/aborted.
 *
 * @param pos	The powered absolute position to move the actuator to (must be
 *		valid).
 */
static inline void
kp_act_start_move_to(kp_act_pos pos)
{
	assert(kp_act_pos_is_valid(pos));
	assert(kp_act_is_initialized());
	kp_act_start_move(kp_act_pos_get_power(pos),
			  false, kp_act_pos_get_steps(pos));
}

/**
 * Move the actuator by a specified number of steps.
 *
 * @param power	The power with which to move the actuator (must be valid).
 * @param steps	The number of steps to move by: positive values
 * 		- lower, negative - higher.
 *
 * @return The movement result.
 */
static inline enum kp_act_move_rc
kp_act_move_by(kp_act_power power, int32_t steps)
{
	assert(kp_act_power_is_valid(power));
	assert(kp_act_is_initialized());
	return kp_act_move(power, true, steps);
}

/**
 * Start moving the actuator by a specified number of steps.
 *
 * @param power	The power with which to move the actuator (must be valid).
 * @param steps	The number of steps to move by: positive values
 * 		- lower, negative - higher.
 *
 * @return The movement result.
 */
static inline void
kp_act_start_move_by(kp_act_power power, int32_t steps)
{
	assert(kp_act_power_is_valid(power));
	assert(kp_act_is_initialized());
	kp_act_start_move(power, true, steps);
}

/**
 * Abort the actuator's movement in progress, if any.
 *
 * @param power	The power with which the actuator is moved.
 *
 * @return True if there was no movement or it was aborted,
 * 	   false if actuator was not powered.
 */
extern bool kp_act_abort(kp_act_power power);

#ifdef __cplusplus
}
#endif

#endif /* KP_ACT_H_ */
