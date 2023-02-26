/** @file
 *  @brief Keypecker capturer
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_CAP_H_
#define KP_CAP_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <stm32_ll_tim.h>
#include <stdbool.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Timer resolution, microseconds */
#define KP_CAP_RES_US	20

/** Maximum time capture of all selected channels can take, us */
#define KP_CAP_TIME_MAX_US	((uint32_t)KP_CAP_RES_US * UINT16_MAX)

/** Number of decimal digits in the maximum capture time, us */
#define KP_CAP_TIME_MAX_DIGITS	7

/** Number of available capture channels */
#define KP_CAP_CH_NUM	2

/** Maximum number of characters in a user's channel name */
#define KP_CAP_CH_NAME_MAX_LEN	15

/** Capture direction sets (bitmaps) */
enum kp_cap_dirs {
	/** No directions (the empty set) */
	KP_CAP_DIRS_NONE	= 0,
	/** Up (a unit set) */
	KP_CAP_DIRS_UP,
	/** Down (a unit set) */
	KP_CAP_DIRS_DOWN,
	/** Both up and down (the universal set) */
	KP_CAP_DIRS_BOTH
};

/** Non-empty capture direction set index */
enum kp_cap_ne_dirs {
	/** Up */
	KP_CAP_NE_DIRS_UP = 0,
	/** Down */
	KP_CAP_NE_DIRS_DOWN,
	/** Both up and down */
	KP_CAP_NE_DIRS_BOTH,
	/** Number of non-empty direction sets (not an index itself) */
	KP_CAP_NE_DIRS_NUM
};

/**
 * Check if a capture direction set is valid.
 *
 * @param dirs	The direction set to check.
 *
 * @return True if the direction set is valid, false otherwise.
 */
static inline bool
kp_cap_dirs_is_valid(enum kp_cap_dirs dirs)
{
	return (dirs & ~KP_CAP_DIRS_BOTH) == 0;
}

/**
 * Check that a capture direction set is a unit set.
 * That is, it contains exactly one member (either UP or DOWN).
 *
 * @param dirs	The direction set to check.
 *
 * @return True if the set is either UP or DOWN.
 */
static bool kp_cap_dirs_is_unit(enum kp_cap_dirs dirs)
{
	assert(kp_cap_dirs_is_valid(dirs));
	return dirs == KP_CAP_DIRS_UP || dirs == KP_CAP_DIRS_DOWN;
}

/**
 * Produce a unit capture direction set from a boolean direction,
 * meaning "down" when true.
 *
 * @param down	True if the direction is down, false if up.
 *
 * @return The generated direction set.
 */
static inline enum kp_cap_dirs kp_cap_dirs_from_down(bool down)
{
	assert((down & 1) == down);
	return KP_CAP_DIRS_UP + down;
}

/**
 * Convert a unit capture direction set to a boolean direction,
 * meaning "down" when true.
 *
 * @param dirs	The unit direction set to convert.
 *
 * @return True if the direction was down, and false if up.
 */
static inline bool kp_cap_dirs_to_down(enum kp_cap_dirs dirs)
{
	assert(kp_cap_dirs_is_unit(dirs));
	return dirs - KP_CAP_DIRS_UP;
}

/**
 * Produce a unit capture direction set from a boolean direction,
 * meaning "up" when true.
 *
 * @param up	True if the direction is up, false if down.
 *
 * @return The generated direction set.
 */
static inline enum kp_cap_dirs kp_cap_dirs_from_up(bool up)
{
	assert((up & 1) == up);
	return KP_CAP_DIRS_DOWN - up;
}

/**
 * Convert a unit capture direction set to a boolean direction,
 * meaning "up" when true.
 *
 * @param dirs	The unit direction set to convert.
 *
 * @return True if the direction was up, and false if down.
 */
static inline bool kp_cap_dirs_to_up(enum kp_cap_dirs dirs)
{
	assert(kp_cap_dirs_is_unit(dirs));
	return KP_CAP_DIRS_DOWN - dirs;
}

/**
 * Convert a direction set to an index of a non-empty direction set.
 *
 * @param dirs	The direction set to convert. Must be a non-empty set.
 *
 * @return The index of the non-empty direction set.
 */
static inline enum kp_cap_ne_dirs kp_cap_dirs_to_ne(enum kp_cap_dirs dirs)
{
	assert(kp_cap_dirs_is_valid(dirs));
	assert(dirs != KP_CAP_DIRS_NONE);
	return (enum kp_cap_ne_dirs)(dirs - 1);
}

/**
 * Convert a non-empty direction set index to a direction set.
 *
 * @param ne_dirs	The index to produce a direction set from.
 *
 * @return The non-empty direction set corresponding to the index.
 */
static inline enum kp_cap_dirs kp_cap_dirs_from_ne(enum kp_cap_ne_dirs ne_dirs)
{
	assert(ne_dirs < KP_CAP_NE_DIRS_NUM);
	return (enum kp_cap_dirs)(ne_dirs + 1);
}

/**
 * Produce the index of a unit capture direction set from a boolean direction,
 * meaning "down" when true.
 *
 * @param down	True if the direction is down, false if up.
 *
 * @return The generated direction set index.
 */
static inline enum kp_cap_ne_dirs kp_cap_ne_dirs_from_down(bool down)
{
	assert((down & 1) == down);
	return (enum kp_cap_ne_dirs)down;
}

/**
 * Produce the index of a unit capture direction set from a boolean direction,
 * meaning "up" when true.
 *
 * @param up	True if the direction is up, false if down.
 *
 * @return The generated direction set index.
 */
static inline enum kp_cap_ne_dirs kp_cap_ne_dirs_from_up(bool up)
{
	assert((up & 1) == up);
	return (enum kp_cap_ne_dirs)(KP_CAP_NE_DIRS_DOWN - up);
}

/**
 * Convert a string to a capture direction set (regardless of case).
 *
 * @param str	The string to convert.
 * @param pdirs	Location for the converted direction set (if valid).
 * 		Can be NULL to discard the converted direction set.
 *
 * @return True if the string was valid and the direction set was output,
 *         False, if the string was invalid and the direction set was not
 *         output.
 */
extern bool kp_cap_dirs_from_str(const char *str, enum kp_cap_dirs *pdirs);

/**
 * Convert a capture direction set to a lower-case string.
 *
 * @param dirs	The direction set to convert.
 *
 * @return The string representing the direction set.
 */
extern const char *kp_cap_dirs_to_lcstr(enum kp_cap_dirs dirs);

/**
 * Convert a capture direction set to a capitalized string.
 *
 * @param dirs	The direction set to convert.
 *
 * @return The string representing the direction set.
 */
extern const char *kp_cap_dirs_to_cpstr(enum kp_cap_dirs dirs);

/** Capture channel configuration */
struct kp_cap_ch_conf {
	/** The movement directions to capture in (bitmap) */
	enum kp_cap_dirs dirs;

	/**
	 * True if the capture should happen on rising edges.
	 * False if it should happen on falling edges.
	 */
	bool rising;

	/** User's channel name */
	char name[KP_CAP_CH_NAME_MAX_LEN + 1];
};

/** Channel capture status */
enum kp_cap_ch_status {
	/** Capture timed out */
	KP_CAP_CH_STATUS_TIMEOUT,
	/** Capture succesfull */
	KP_CAP_CH_STATUS_OK,
	/** More than one capture event occurred */
	KP_CAP_CH_STATUS_OVERCAPTURE,
	/** Number of statuses - not a valid status itself */
	KP_CAP_CH_STATUS_NUM
};

/**
 * Check if a channel status is valid.
 *
 * @param status	The status to check.
 *
 * @return True if the status is valid, false otherwise.
 */
static inline bool
kp_cap_ch_status_is_valid(enum kp_cap_ch_status status)
{
	return status >= 0 && status < KP_CAP_CH_STATUS_NUM;
}

/**
 * Convert a channel's capture status to string.
 *
 * @param status	The status to convert.
 *
 * @return The string representing the status.
 */
extern const char *kp_cap_ch_status_to_str(enum kp_cap_ch_status status);

/** Channel capture result */
struct kp_cap_ch_res {
	/** Capture status */
	enum kp_cap_ch_status status:2;
	/**
	 * Captured time value.
	 * Only valid if status is OK or OVERCAPTURE.
	 */
	uint32_t value_us:30;
};

/**
 * The ISR for UP/CC timer interrupts.
 *
 * @param arg	Unused.
 */
extern void kp_cap_isr(void *arg);


/** Debug output configuration */
struct kp_cap_dbg_conf {
	/*
	 * The GPIO port to use for debug event output.
	 * NULL to have debugging disabled.
	 */
	const struct device *gpio;

	/*
	 * The GPIO pin to use for update interrupt debugging.
	 * Set high at the trigger, set low on update.
	 * Must be configured. Set to UINT8_MAX to disable.
	 */
	gpio_pin_t update_pin;

	/**
	 * The GPIO pins to use for capture interrupt debugging. One per
	 * channel. Each set high at the trigger, set low when the channel is
	 * captured. Each must be configured. Any can be set to UINT8_MAX to
	 * disable.
	 */
	gpio_pin_t cap_pin_list[KP_CAP_CH_NUM];
};

/**
 * Initialize the capturer.
 *
 * @param timer		The STM32 timer to use for capturing.
 * 			The timer's rising CH1 input will be used to start
 * 			counting, and the CH2-CH3 channels to capture events,
 * 			as configured when starting the capture.
 * @param dbg_conf	Debug output configuration.
 *			NULL to have debugging output disabled.
 */
extern void kp_cap_init(TIM_TypeDef* timer,
			const struct kp_cap_dbg_conf *dbg_conf);

/**
 * Check if the capturer is initialized.
 *
 * @return True if the capturer is initialized, false if not.
 */
extern bool kp_cap_is_initialized(void);

/** Capture configuration */
struct kp_cap_conf {
	/** Channel configurations */
	struct kp_cap_ch_conf ch_list[KP_CAP_CH_NUM];
	/**
	 * The maximum time to wait for all channels to be captured,
	 * microseconds. Must not be greater than KP_CAP_TIME_MAX_US -
	 * bounce_us.
	 */
	uint32_t timeout_us;
	/**
	 * The minimum time to wait for a channel to bounce, microseconds.
	 * Must not be greater than KP_CAP_TIME_MAX_US - timeout_us.
	 */
	uint32_t bounce_us;
};

/**
 * Check that a capture configuration is valid.
 *
 * @param conf	The configuration to check.
 *
 * @return True if the configuration is valid, false otherwise.
 */
static inline bool
kp_cap_conf_is_valid(const struct kp_cap_conf *conf)
{
	return conf != NULL &&
	       (conf->timeout_us + conf->bounce_us) <= KP_CAP_TIME_MAX_US;
}

/**
 * Get the number of channels enabled in a configuration for the specified
 * directions.
 *
 * @param conf	The capture configuration to count the channels in.
 * @param dirs	The directions to count the channels in. Use KP_CAP_DIRS_BOTH
 * 		to count all enabled channels, regardless of direction.
 */
extern size_t kp_cap_conf_ch_num(const struct kp_cap_conf *conf,
				 enum kp_cap_dirs dirs);

/**
 * Calculate the index of a channel capture result in a flat result array,
 * for specified configuration, current pass number, the direction of even
 * passes, and the channel number.
 *
 * @param conf		The configuration the capture was done with.
 * @param even_down	True if even passes are directed down, false if up.
 * @param pass		The index of the pass the channel result is in.
 * @param ch		The index of the channel to get index for, within the
 *			pass. Must be less than the number of channels in the
 *			configuration.
 */
extern size_t kp_cap_conf_ch_res_idx(const struct kp_cap_conf *conf,
				     bool even_down,
				     size_t pass,
				     size_t ch);

/**
 * Start capture, waiting for the previous one to finish first.
 *
 * @param conf	Capture configuration to use.
 * @param dirs	The movement directions the capture is happening in.
 */
extern void kp_cap_start(const struct kp_cap_conf *conf, enum kp_cap_dirs dirs);

/**
 * Initialize a poll event to wait for finished captures.
 *
 * @param event	The poll event to initialize.
 */
extern void kp_cap_finish_event_init(struct k_poll_event *event);

/** Capture result code */
enum kp_cap_rc {
	/** Capture succeeded, check individual channel results */
	KP_CAP_RC_OK,
	/** Capture was aborted, channel results not modified */
	KP_CAP_RC_ABORTED,
	/** Waiting for capture timed out, channel results not modified */
	KP_CAP_RC_TIMEOUT,
	/** Number of result codes, not a valid result code itself */
	KP_CAP_RC_NUM
};

/**
 * Check if a capture result code is valid.
 *
 * @param rc	The result code to check.
 *
 * @return True if the result code is valid, false otherwise.
 */
static inline bool
kp_cap_rc_is_valid(enum kp_cap_rc rc)
{
	return rc >= 0 && rc < KP_CAP_RC_NUM;
}

/**
 * Convert a capture result code to string.
 *
 * @param rc	The result code to convert.
 *
 * @return The string representing the result code.
 */
extern const char *kp_cap_rc_to_str(enum kp_cap_rc rc);

/**
 * Abort the current capture, if running.
 *
 * @return True if the capture was running and aborted, false otherwise.
 */
extern bool kp_cap_abort(void);

/**
 * Retrieve capture results when/if they're ready.
 *
 * @param ch_res_list	List of structures for channel capture results.
 * 			Only modified if KP_CAP_RC_OK is returned.
 * 			Only results for channels enabled in the configuration
 * 			and the directions passed to kp_cap_start() (as
 * 			counted by kp_cap_conf_ch_num()) will be output. Can
 * 			be NULL if ch_res_num is zero.
 * @param ch_res_num	Maximum number of channels to retrieve results for.
 * @param timeout	The time to wait for the capture to finish, or one of
 * 			the special values K_NO_WAIT and K_FOREVER.
 *
 * @return The capture result code.
 */
extern enum kp_cap_rc kp_cap_finish(struct kp_cap_ch_res *ch_res_list,
				    size_t ch_res_num, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* KP_CAP_H_ */
