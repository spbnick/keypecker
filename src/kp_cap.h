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

/** Capture channel configuration */
struct kp_cap_ch_conf {
	/** True if the channel should be captured */
	bool capture;
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
	/** Capture not enabled */
	KP_CAP_CH_STATUS_DISABLED = 0,
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
	enum kp_cap_ch_status status:4;
	/**
	 * Captured time value.
	 * Only valid if status is OK or OVERCAPTURE.
	 */
	uint32_t value_us:28;
};

/**
 * The ISR for UP/CC timer interrupts.
 *
 * @param arg	Unused.
 */
extern void kp_cap_isr(void *arg);

/**
 * Initialize the capturer.
 *
 * @param timer	The STM32 timer to use for capturing.
 * 		The timer's rising CH1 input will be used to start counting,
 * 		and the CH2-CH3 channels to capture events, as configured
 * 		after initialization.
 */
extern void kp_cap_init(TIM_TypeDef* timer);

/**
 * Check if the capturer is initialized.
 *
 * @return True if the capturer is initialized, false if not.
 */
extern bool kp_cap_is_initialized(void);

/**
 * Start capture, waiting for the previous one to finish first.
 *
 * @param ch_conf_list	An array configurations for channels to be captured.
 * @param ch_conf_num	Number of configurations in the "ch_conf_list".
 * 			Any channels above this number won't be captured.
 * 			Any configurations above KP_CAP_CH_NUM will be
 * 			ignored.
 * @param timeout_us	The maximum time to wait for all channels to be
 * 			captured, microseconds. Must not be greater than
 * 			KP_CAP_TIME_MAX_US - bounce_us.
 * @param bounce_us	The minimum time to wait for a channel to bounce,
 *			microseconds. Must not be greater than
 * 			KP_CAP_TIME_MAX_US - timeout_us.
 */
extern void kp_cap_start(const struct kp_cap_ch_conf *ch_conf_list,
			 size_t ch_conf_num, uint32_t timeout_us,
			 uint32_t bounce_us);


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
 * @param ch_res_num	Number of channels to retrieve results for.
 * 			Result structures above KP_CAP_CH_NUM will be
 * 			unmodified.
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
