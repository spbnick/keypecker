/** @file
 *  @brief Keypecker input
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_INPUT_H_
#define KP_INPUT_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Input messages */
enum kp_input_msg {
	/** Abort (Ctrl-C) */
	KP_INPUT_MSG_ABORT,
	/** Up arrow */
	KP_INPUT_MSG_UP,
	/** Down arrow */
	KP_INPUT_MSG_DOWN,
	/** Enter */
	KP_INPUT_MSG_ENTER,
};

/**
 * Reset tracked input state to start processing another session.
 */
extern void kp_input_reset(void);

/**
 * Receive raw input for processing.
 *
 * @param data  Raw data from transport.
 * @param len   Data length.
 */
extern void kp_input_recv(uint8_t *data, size_t len);

/**
 * Initialize a poll event to wait for input.
 *
 * @param event	The poll event to initialize.
 */
extern void kp_input_get_event_init(struct k_poll_event *event);

/**
 * Get the next input message.
 *
 * @param msg		Location for the retrieved message.
 * @param timeout	The time to wait for the message to arrive, or one of
 * 			the special values K_NO_WAIT and K_FOREVER.
 *
 * @retval 0 Message received.
 * @retval -ENOMSG Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 */
extern int kp_input_get(enum kp_input_msg *msg, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* KP_INPUT_H_ */
