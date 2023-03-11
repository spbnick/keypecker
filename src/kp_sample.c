/** @file
 *  @brief Keypecker sample handling
 */

/*
 * Copyright (c) 2023 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_sample.h"
#include <string.h>
#include <stdlib.h>

enum kp_sample_rc
kp_sample(int32_t target,
	  uint32_t speed,
	  const struct kp_cap_conf *conf,
	  enum kp_cap_dirs dirs,
	  struct kp_cap_ch_res *ch_res_list,
	  size_t ch_res_num)
{
	/* Poll event indices */
	enum {
		EVENT_IDX_INPUT = 0,
		EVENT_IDX_ACT_FINISH_MOVE,
		EVENT_IDX_CAP_FINISH,
		EVENT_NUM
	};
	struct k_poll_event events[EVENT_NUM];
	enum kp_act_move_rc move_rc = KP_ACT_MOVE_RC_OK;
	enum kp_cap_rc cap_rc = KP_CAP_RC_OK;
	int32_t start;
	bool moved = false;
	bool captured = false;
	enum kp_input_msg msg;
	size_t i;

	assert(kp_act_pos_is_valid(target));
	assert(kp_cap_conf_is_valid(conf));
	assert(ch_res_list != NULL || ch_res_num == 0);

	/* Get the start actuator position */
	start = kp_act_locate();
	/* If the actuator is off */
	if (!kp_act_pos_is_valid(start)) {
		return KP_SAMPLE_RC_OFF;
	}

	/* If we are not going to move, and so won't trigger a capture */
	if (target == start) {
		/* Output timeouts for all channels */
		memset(ch_res_list, 0, sizeof(*ch_res_list) * ch_res_num);
		return KP_SAMPLE_RC_OK;
	}

	/* Initialize events */
	kp_input_get_event_init(&events[EVENT_IDX_INPUT]);
	kp_act_finish_move_event_init(&events[EVENT_IDX_ACT_FINISH_MOVE]);
	kp_cap_finish_event_init(&events[EVENT_IDX_CAP_FINISH]);

	/* Start the capture */
	kp_cap_start(conf, dirs);

	/* Start moving towards the target */
	kp_act_start_move_to(target, speed);

	/* Move and capture */
	for (; !moved || !captured;) {
		while (k_poll(events, ARRAY_SIZE(events), K_FOREVER) != 0);

		/* Handle input */
		if (events[EVENT_IDX_INPUT].state) {
			while (kp_input_get(&msg, K_FOREVER) != 0);
			if (msg == KP_INPUT_MSG_ABORT) {
				kp_act_abort();
				kp_cap_abort();
			}
		}

		/* Handle movement completion */
		if (events[EVENT_IDX_ACT_FINISH_MOVE].state) {
			move_rc = kp_act_finish_move(K_FOREVER);
			moved = true;
		}

		/* Handle capture completion */
		if (events[EVENT_IDX_CAP_FINISH].state) {
			cap_rc = kp_cap_finish(ch_res_list, ch_res_num,
					       K_FOREVER);
			captured = true;
		}

		/* Reset event state */
		for (i = 0; i < ARRAY_SIZE(events); i++) {
			events[i].state = K_POLL_STATE_NOT_READY;
		}
	}

	if (move_rc == KP_ACT_MOVE_RC_ABORTED ||
			cap_rc == KP_CAP_RC_ABORTED) {
		return KP_SAMPLE_RC_ABORTED;
	}
	if (move_rc == KP_ACT_MOVE_RC_OFF) {
		return KP_SAMPLE_RC_OFF;
	}
	assert(move_rc == KP_ACT_MOVE_RC_OK);
	assert(cap_rc == KP_CAP_RC_OK);

	return KP_SAMPLE_RC_OK;
}

enum kp_sample_rc
kp_sample_check(int32_t top, int32_t bottom,
		uint32_t speed, size_t passes,
		const struct kp_cap_conf *conf,
		size_t *ptriggers)
{
	struct kp_cap_ch_res ch_res_list[KP_CAP_CH_NUM];
	enum kp_sample_rc rc = KP_CAP_RC_OK;
	int32_t pos;
	bool even_down;
	size_t pass;
	size_t captured_pass;
	size_t triggers = 0;
	size_t i;
	size_t captured_channels;
	size_t triggered_channels;

	assert(kp_act_pos_is_valid(top));
	assert(kp_act_pos_is_valid(bottom));
	assert(kp_cap_conf_is_valid(conf));
	assert(kp_cap_conf_ch_num(conf, KP_CAP_DIRS_BOTH) > 0);

	if (passes == 0) {
		goto finish;
	}

	/* Move to the closest boundary without capturing */
	pos = kp_act_locate();
	if (!kp_act_pos_is_valid(pos)) {
		return KP_SAMPLE_RC_OFF;
	}
	even_down = abs(pos - top) < abs(pos - bottom);
	rc = kp_sample(even_down ? top : bottom,
		       speed, conf, KP_CAP_DIRS_NONE, NULL, 0);
	if (rc != KP_SAMPLE_RC_OK) {
		return rc;
	}

	for (pass = 0, captured_pass = 0; captured_pass < passes; pass++) {
		enum kp_cap_dirs dirs =
			kp_cap_dirs_from_down(even_down ^ (pass & 1));
		/* Capture moving to the opposite boundary */
		rc = kp_sample(
			(dirs == KP_CAP_DIRS_DOWN) ? bottom : top, speed,
			conf, dirs, ch_res_list, ARRAY_SIZE(ch_res_list)
		);
		if (rc != KP_SAMPLE_RC_OK) {
			return rc;
		}

		/* Check if any and all captured channels triggered */
		for (i = 0, captured_channels = 0, triggered_channels = 0;
		     i < ARRAY_SIZE(conf->ch_list); i++) {
			/* If the channel is enabled in this direction */
			if (conf->ch_list[i].dirs & dirs) {
				if (ch_res_list[captured_channels].status !=
						KP_CAP_CH_STATUS_TIMEOUT) {
					triggered_channels++;
				}
				captured_channels++;
			}
		}
		if (captured_channels > 0) {
			captured_pass++;
			if (triggered_channels == captured_channels) {
				triggers++;
			}
		}
	}

finish:

	if (ptriggers != NULL) {
		*ptriggers = triggers;
	}

	return KP_SAMPLE_RC_OK;
}

