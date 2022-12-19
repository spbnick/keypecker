/** @file
 *  @brief Keypecker input
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_input.h"

/** The mutex protecting all the state */
static K_MUTEX_DEFINE(kp_input_mutex);

/** Input message queue */
K_MSGQ_DEFINE(kp_input_msgq, sizeof(enum kp_input_msg),
		16, sizeof(enum kp_input_msg));

/** Input state */
enum kp_input_st {
	/** Base state, no special characters encountered */
	KP_INPUT_ST_NONE,
	/** Escape character encountered */
	KP_INPUT_ST_ESC,
	/** Control Sequence Introducer (CSI) encountered */
	KP_INPUT_ST_CSI,
	/** Control sequence intermediate byte(s) received */
	KP_INPUT_ST_CSI_INT,
};

/** Current input state */
static enum kp_input_st kp_input_st;

void
kp_input_reset(void)
{
	k_mutex_lock(&kp_input_mutex, K_FOREVER);
	kp_input_st = KP_INPUT_ST_NONE;
	k_msgq_purge(&kp_input_msgq);
	k_mutex_unlock(&kp_input_mutex);
}

void
kp_input_recv(uint8_t *data, size_t len)
{
	enum kp_input_msg msg;
	k_mutex_lock(&kp_input_mutex, K_FOREVER);
	for (; len > 0; data++, len--) {
		switch (kp_input_st) {
		/* Base state */
		case KP_INPUT_ST_NONE:
			switch(*data) {
			case 0x03: /* ETX (Ctrl-C) */
				msg = KP_INPUT_MSG_ABORT;
				k_msgq_put(&kp_input_msgq, &msg, K_FOREVER);
				k_mutex_unlock(&kp_input_mutex);
				return;
			case 0x0d: /* CR (Enter) */
				msg = KP_INPUT_MSG_ENTER;
				k_msgq_put(&kp_input_msgq, &msg, K_FOREVER);
				break;
			case 0x1b: /* ESC  */
				kp_input_st = KP_INPUT_ST_ESC;
				break;
			}
			break;
		/* Esc received */
		case KP_INPUT_ST_ESC:
			switch (*data) {
			case '[': /* CSI */
				kp_input_st = KP_INPUT_ST_CSI;
				break;
			default:
				/* Unknown or invalid ESC sequence */
				kp_input_st = KP_INPUT_ST_NONE;
				break;
			}
			break;
		/* If an intermediate byte was received after a CSI */
		case KP_INPUT_ST_CSI_INT:
			/* If a parameter byte is received */
			if ((*data >> 4) == 3) {
				/* Invalid ESC sequence */
				kp_input_st = KP_INPUT_ST_NONE;
				break;
			}
			/* Fallthrough */
		/* CSI has been received */
		case KP_INPUT_ST_CSI:
			switch (*data) {
			case 'A': /* Up arrow */
				msg = KP_INPUT_MSG_UP;
				k_msgq_put(&kp_input_msgq, &msg, K_FOREVER);
				kp_input_st = KP_INPUT_ST_NONE;
				break;
			case 'B': /* Down arrow */
				msg = KP_INPUT_MSG_DOWN;
				k_msgq_put(&kp_input_msgq, &msg, K_FOREVER);
				kp_input_st = KP_INPUT_ST_NONE;
				break;
			default:
				/* If it's an intermediate byte */
				if ((*data >> 4) == 2) {
					kp_input_st = KP_INPUT_ST_CSI_INT;
				/* If it's anything but a parameter byte */
				} else if ((*data >> 4) != 3) {
					/* Finished or invalid ESC sequence */
					kp_input_st = KP_INPUT_ST_NONE;
				}
				break;
			}
			break;
		}

	}
	k_mutex_unlock(&kp_input_mutex);
}

void
kp_input_get_event_init(struct k_poll_event *event)
{
	k_poll_event_init(event, K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
			  K_POLL_MODE_NOTIFY_ONLY, &kp_input_msgq);
}

int
kp_input_get(enum kp_input_msg *msg, k_timeout_t timeout)
{
	return k_msgq_get(&kp_input_msgq, msg, timeout);
}
