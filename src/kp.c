/** @file
 *  @brief Keypecker main entry point
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_cap.h"
#include "kp_act.h"
#include "kp_shell.h"
#include "kp_input.h"
#include "kp_misc.h"
#include <stm32_ll_tim.h>
#include <assert.h>
#include <stdlib.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>

/** Devicetree node identifier for the actuator's GPIO port */
#define KP_ACT_GPIO_NODE DT_NODELABEL(gpiob)
/** Devicetree node identifier for the debug GPIO port */
#define KP_DBG_GPIO_NODE DT_NODELABEL(gpioa)

/** Devicetree node identifier for the timer */
#define KP_TIMER_NODE DT_NODELABEL(timers1)

/** The actuator GPIO port device */
static const struct device *kp_act_gpio = DEVICE_DT_GET(KP_ACT_GPIO_NODE);

/** The debug GPIO port device */
static const struct device *kp_dbg_gpio = DEVICE_DT_GET(KP_DBG_GPIO_NODE);

/** The pin for update interrupt debugging */
const gpio_pin_t kp_dbg_pin_update = 3;

/** The base pin for channel capture debugging */
const gpio_pin_t kp_dbg_pin_ch_base = 4;

/** Actuator speed, 0-100% */
static uint32_t kp_act_speed = 100;

/** Top actuator position */
static int32_t kp_act_pos_top = KP_ACT_POS_INVALID;

/** Bottom actuator position */
static int32_t kp_act_pos_bottom = KP_ACT_POS_INVALID;

/** Execute the "on" command */
static int
kp_cmd_on(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!kp_act_on()) {
		shell_info(shell, "Actuator is already on");
	}
	return 0;
}

SHELL_CMD_REGISTER(on, NULL, "Turn on actuator", kp_cmd_on);

/** Execute the "off" command */
static int
kp_cmd_off(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (kp_act_off()) {
		kp_act_pos_top = KP_ACT_POS_INVALID;
		kp_act_pos_bottom = KP_ACT_POS_INVALID;
	} else {
		shell_info(shell, "Actuator is already off");
	}
	return 0;
}

SHELL_CMD_REGISTER(off, NULL, "Turn off actuator", kp_cmd_off);

/**
 * Parse a non-negative number from a string.
 *
 * @param str	The string to parse the number from.
 * @param pn	Location for the number parsed from the string.
 *
 * @return True if the string was representing a valid non-negative number,
 * 		false otherwise.
 */
static bool
kp_parse_non_negative_number(const char *str, long *pn)
{
	char *end;
	long n;
	n = strtol(str, &end, 10);
	if (*str == '\0' || *end != '\0' || n < 0 || n == LONG_MAX) {
		return false;
	}
	*pn = n;
	return true;
}

/** Execute the "up [steps]" command */
static int
kp_cmd_up(const struct shell *shell, size_t argc, char **argv)
{
	long steps;
	enum kp_act_move_rc rc;
	if (argc >= 2) {
		if (!kp_parse_non_negative_number(argv[1], &steps)) {
			shell_error(shell, "Invalid number of steps: %s",
					argv[1]);
			return 1;
		}
	} else {
		steps = 1;
	}
	rc = kp_act_move_by(-steps, kp_act_speed);
	switch (rc) {
		case KP_ACT_MOVE_RC_OFF:
			shell_error(shell, "Actuator is off, stopping");
			break;
		case KP_ACT_MOVE_RC_ABORTED:
			shell_error(shell, "Aborted");
			break;
		default:
			break;
	}
	return rc != KP_ACT_MOVE_RC_OK;
}

SHELL_CMD_ARG_REGISTER(up, NULL, "Move actuator up (n steps)",
			kp_cmd_up, 1, 1);

/** Execute the "down [steps]" command */
static int
kp_cmd_down(const struct shell *shell, size_t argc, char **argv)
{
	long steps;
	enum kp_act_move_rc rc;
	if (argc >= 2) {
		if (!kp_parse_non_negative_number(argv[1], &steps)) {
			shell_error(shell, "Invalid number of steps: %s",
					argv[1]);
			return 1;
		}
	} else {
		steps = 1;
	}
	rc = kp_act_move_by(steps, kp_act_speed);
	switch (rc) {
		case KP_ACT_MOVE_RC_OFF:
			shell_error(shell, "Actuator is off, stopping");
			break;
		case KP_ACT_MOVE_RC_ABORTED:
			shell_error(shell, "Aborted");
			break;
		default:
			break;
	}
	return rc != KP_ACT_MOVE_RC_OK;
}

SHELL_CMD_ARG_REGISTER(down, NULL, "Move actuator down (n steps)",
			kp_cmd_down, 1, 1);

/**
 * Process input from the bypassed shell of a scheduled command.
 *
 * @param shell Shell instance.
 * @param data  Raw data from transport.
 * @param len   Data length.
 */
static void
kp_input_bypass_cb(const struct shell *shell, uint8_t *data, size_t len)
{
	ARG_UNUSED(shell);
	kp_input_recv(data, len);
}

/** Execute the "swing steps" command */
static int
kp_cmd_swing(const struct shell *shell, size_t argc, char **argv)
{
	/* Poll event indices */
	enum {
		EVENT_IDX_INPUT = 0,
		EVENT_IDX_ACT_FINISH_MOVE,
		EVENT_NUM
	};
	long steps;
	enum kp_act_move_rc rc;
	enum kp_input_msg msg;
	int32_t start_pos;
	struct k_poll_event events[EVENT_NUM];

	/* Parse arguments */
	assert(argc == 2 || argc == (size_t)SSIZE_MAX + 2);
	if (!kp_parse_non_negative_number(argv[1], &steps) || steps == 0) {
		shell_error(shell, "Invalid number of steps: %s",
				argv[1]);
		return 1;
	}

	/* Check for power */
	if (kp_act_is_off()) {
		shell_error(shell, "Actuator is off, aborting");
		return 1;
	}

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_swing, kp_input_bypass_cb);
	kp_input_reset();

	/* Remember the start position */
	start_pos = kp_act_locate();

	/* Initialize events */
	kp_input_get_event_init(&events[EVENT_IDX_INPUT]);
	kp_act_finish_move_event_init(&events[EVENT_IDX_ACT_FINISH_MOVE]);

	/* Move */
	shell_print(shell, "Swinging, press Enter to stop, Ctrl-C to abort");
	rc = kp_act_move_by(steps / 2, kp_act_speed);
	bool finished = false;
	while (rc == KP_ACT_MOVE_RC_OK && !finished) {
		bool moved = false;
		steps = -steps;
		kp_act_start_move_by(steps, kp_act_speed);
		while (rc == KP_ACT_MOVE_RC_OK && !moved) {
			while (k_poll(events, ARRAY_SIZE(events), K_FOREVER) != 0);

			if (events[EVENT_IDX_INPUT].state) {
				while (kp_input_get(&msg, K_FOREVER) != 0);
				if (msg == KP_INPUT_MSG_ABORT) {
					kp_act_abort();
				} else if (msg == KP_INPUT_MSG_ENTER) {
					finished = true;
				}
			}

			if (events[EVENT_IDX_ACT_FINISH_MOVE].state) {
				rc = kp_act_finish_move(K_FOREVER);
				moved = (rc == KP_ACT_MOVE_RC_OK);
			}

			/* Reset event state */
			for (size_t i = 0; i < ARRAY_SIZE(events); i++) {
				events[i].state = K_POLL_STATE_NOT_READY;
			}
		}
	}

	if (finished && rc == KP_ACT_MOVE_RC_OK) {
		/* Return to the start position */
		rc = kp_act_move_to(start_pos, kp_act_speed);
	}

	/* Report error, if any */
	if (rc == KP_ACT_MOVE_RC_ABORTED) {
		shell_error(shell, "Aborted");
	} else if (rc == KP_ACT_MOVE_RC_OFF) {
		shell_error(shell, "Actuator is off, stopping");
	}
	return rc != KP_ACT_MOVE_RC_OK;
}

SHELL_CMD_ARG_REGISTER(swing, NULL,
			"Move actuator back-n-forth within n steps around "
			"current position, until interrupted",
			kp_cmd_swing, 2, 0);

/** Execute the "adjust" command */
static int
kp_cmd_adjust(const struct shell *shell, size_t argc, char **argv)
{
	enum kp_act_move_rc rc;
	enum kp_input_msg msg;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Check for power */
	if (kp_act_is_off()) {
		shell_error(shell, "Actuator is off, aborting");
		return 1;
	}

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_adjust, kp_input_bypass_cb);
	kp_input_reset();

	/* Move */
	shell_print(shell,
		    "Press up and down arrow keys to move the actuator.");
	shell_print(shell, "Press Enter to stop, Ctrl-C to abort.");
	do {
		/* Read next input message */
		while (kp_input_get(&msg, K_FOREVER) != 0);
		if (msg == KP_INPUT_MSG_UP || msg == KP_INPUT_MSG_DOWN) {
			rc = kp_act_move_by(
				(msg == KP_INPUT_MSG_DOWN) ? 1 : -1,
				kp_act_speed
			);
		} else if (msg == KP_INPUT_MSG_ABORT) {
			rc = KP_ACT_MOVE_RC_ABORTED;
		} else if (msg == KP_INPUT_MSG_ENTER) {
			break;
		} else {
			rc = KP_ACT_MOVE_RC_OK;
		}
	} while (rc == KP_ACT_MOVE_RC_OK);

	/* Report error, if any */
	if (rc == KP_ACT_MOVE_RC_ABORTED) {
		shell_error(shell, "Aborted");
	} else if (rc == KP_ACT_MOVE_RC_OFF) {
		shell_error(shell, "Actuator is off, stopping");
	}
	return rc != KP_ACT_MOVE_RC_OK;
}

SHELL_CMD_REGISTER(adjust, NULL, "Adjust actuator position interactively",
			kp_cmd_adjust);

/** Execute the "set speed <percentage>" command */
static int
kp_cmd_set_speed(const struct shell *shell, size_t argc, char **argv)
{
	long speed;

	assert(argc == 2);

	if (!kp_parse_non_negative_number(argv[1], &speed) || speed > 100) {
		shell_error(
			shell,
			"Invalid speed percentage (expecting 0-100): %s",
			argv[1]
		);
		return 1;
	}
	kp_act_speed = (uint32_t)speed;
	return 0;
}

/** Execute the "set top" command */
static int
kp_cmd_set_top(const struct shell *shell, size_t argc, char **argv)
{
	int32_t pos;
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	pos = kp_act_locate();
	if (!kp_act_pos_is_valid(pos)) {
		shell_error(shell, "Actuator is off, position not set");
		return 1;
	}
	if (kp_act_pos_is_valid(kp_act_pos_bottom) &&
			pos >= kp_act_pos_bottom) {
		shell_error(shell, "Position not above bottom, not set");
		return 1;
	}
	kp_act_pos_top = pos;
	return 0;
}

/** Execute the "set bottom" command */
static int
kp_cmd_set_bottom(const struct shell *shell, size_t argc, char **argv)
{
	int32_t pos;
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	pos = kp_act_locate();
	if (!kp_act_pos_is_valid(pos)) {
		shell_error(shell, "Actuator is off, position not set");
		return 1;
	}
	if (kp_act_pos_is_valid(kp_act_pos_top) &&
			pos <= kp_act_pos_top) {
		shell_error(shell, "Position not below top, not set");
		return 1;
	}
	kp_act_pos_bottom = pos;
	return 0;
}

/** Capture channel configurations */
static struct kp_cap_ch_conf kp_cap_ch_conf_list[KP_CAP_CH_NUM];

/* List of capture result lists */
static struct kp_cap_ch_res kp_cap_ch_res_list_list[512][KP_CAP_CH_NUM];

/** Capture timeout, us */
static uint32_t kp_cap_timeout_us = 1000000;

/** Bounce time, us */
static uint32_t kp_cap_bounce_us = 50000;

/** Execute the
 * "set ch <idx> none/up/down/both [rising/falling [<name>]]"
 * command */
static int
kp_cmd_set_ch(const struct shell *shell, size_t argc, char **argv)
{
	long idx;
	const char *arg;
	struct kp_cap_ch_conf conf;
	assert(argc >= 3);
	assert(argc <= 5);

	/* Parse channel index */
	arg = argv[1];
	if (!kp_parse_non_negative_number(arg, &idx) ||
			idx >= ARRAY_SIZE(kp_cap_ch_conf_list)) {
		shell_error(
			shell,
			"Invalid channel index (0-%zu expected): %s",
			ARRAY_SIZE(kp_cap_ch_conf_list) - 1,
			arg
		);
		return 1;
	}

	/* Read current state */
	conf = kp_cap_ch_conf_list[idx];

	/* Parse capture direction, if specified */
	if (argc >= 3) {
		arg = argv[2];
		if (!kp_cap_dir_from_str(arg, &conf.dir)) {
			shell_error(
				shell,
				"Invalid capture direction "
				"(none/up/down/both expected): %s",
				arg
			);
			return 1;
		}
	}

	/* Parse capture edge, if specified */
	if (argc >= 4) {
		arg = argv[3];
		if (kp_strcasecmp(arg, "rising") == 0) {
			conf.rising = true;
		} else if (kp_strcasecmp(arg, "falling") == 0) {
			conf.rising = false;
		} else {
			shell_error(
				shell,
				"Invalid capture edge "
				"(rising/falling expected): %s",
				arg
			);
			return 1;
		}
	}

	/* Read name, if specified */
	if (argc >= 5) {
		arg = argv[4];
		if (strlen(arg) >= sizeof(conf.name)) {
			shell_error(
				shell,
				"Channel name too long "
				"(%zu > %zu expected characters): %s",
				strlen(arg),
				sizeof(conf.name) - 1,
				arg
			);
			return 1;
		}
		strcpy(conf.name, arg);
	}

	/* Store the parameters */
	kp_cap_ch_conf_list[idx] = conf;

	return 0;
}

/** Execute the "set timeout <us>" command */
static int
kp_cmd_set_timeout(const struct shell *shell, size_t argc, char **argv)
{
	long timeout_us;

	assert(argc == 2);

	if (!kp_parse_non_negative_number(argv[1], &timeout_us)) {
		shell_error(shell, "Invalid timeout: %s", argv[1]);
		return 1;
	}
	if ((uint32_t)timeout_us + kp_cap_bounce_us >= KP_CAP_TIME_MAX_US) {
		shell_error(
			shell,
			"Timeout plus bounce time exceed "
			"maximum capture time: %ld + %u >= %u",
			timeout_us, kp_cap_bounce_us, KP_CAP_TIME_MAX_US
		);
		return 1;
	};
	kp_cap_timeout_us = (uint32_t)timeout_us;
	return 0;
}

/** Execute the "set bounce <us>" command */
static int
kp_cmd_set_bounce(const struct shell *shell, size_t argc, char **argv)
{
	long bounce_us;

	assert(argc == 2);

	if (!kp_parse_non_negative_number(argv[1], &bounce_us)) {
		shell_error(shell, "Invalid bounce time: %s", argv[1]);
		return 1;
	}
	if ((uint32_t)kp_cap_timeout_us + bounce_us >= KP_CAP_TIME_MAX_US) {
		shell_error(
			shell,
			"Bounce time plus timeout exceed "
			"maximum capture time: %ld + %u >= %u",
			bounce_us, kp_cap_timeout_us, KP_CAP_TIME_MAX_US
		);
		return 1;
	};
	kp_cap_bounce_us = (uint32_t)bounce_us;
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(set_subcmds,
	SHELL_CMD_ARG(speed, NULL,
			"Set speed: <percentage>",
			kp_cmd_set_speed, 2, 0),
	SHELL_CMD(top, NULL, "Register current position as the top",
			kp_cmd_set_top),
	SHELL_CMD(bottom, NULL, "Register current position as the bottom",
			kp_cmd_set_bottom),
	SHELL_CMD_ARG(ch, NULL,
			"Set channel configuration: "
			"<idx> none/up/down/both [rising/falling [<name>]]",
			kp_cmd_set_ch, 3, 2),
	SHELL_CMD_ARG(timeout, NULL,
			"Set capture timeout: <us>",
			kp_cmd_set_timeout, 2, 0),
	SHELL_CMD_ARG(bounce, NULL,
			"Set bounce time: <us>",
			kp_cmd_set_bounce, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(set, &set_subcmds,
			"Set parameters", NULL);

/** Execute the "get speed" command */
static int
kp_cmd_get_speed(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(shell, "%u%%", kp_act_speed);
	return 0;
}

/** Execute the "get top" command */
static int
kp_cmd_get_top(const struct shell *shell, size_t argc, char **argv)
{
	enum kp_act_move_rc rc;
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!kp_act_pos_is_valid(kp_act_pos_top)) {
		shell_error(shell, "Top position not set, not moving");
		return 1;
	}
	rc = kp_act_move_to(kp_act_pos_top, kp_act_speed);
	if (rc == KP_ACT_MOVE_RC_ABORTED) {
		shell_error(shell, "Aborted");
	} else if (rc == KP_ACT_MOVE_RC_OFF) {
		shell_error(shell, "Actuator is off, stopping");
	}
	return rc != KP_ACT_MOVE_RC_OK;
}

/** Execute the "get bottom" command */
static int
kp_cmd_get_bottom(const struct shell *shell, size_t argc, char **argv)
{
	enum kp_act_move_rc rc;
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!kp_act_pos_is_valid(kp_act_pos_bottom)) {
		shell_error(shell, "Bottom position not set, not moving");
		return 1;
	}
	rc = kp_act_move_to(kp_act_pos_bottom, kp_act_speed);
	if (rc == KP_ACT_MOVE_RC_ABORTED) {
		shell_error(shell, "Aborted");
	} else if (rc == KP_ACT_MOVE_RC_OFF) {
		shell_error(shell, "Actuator is off, stopping");
	}
	return rc != KP_ACT_MOVE_RC_OK;
}

/** Execute the "get ch <idx>" command */
static int
kp_cmd_get_ch(const struct shell *shell, size_t argc, char **argv)
{
	long idx;
	const char *arg;
	const struct kp_cap_ch_conf *conf;

	assert(argc == 2);

	/* Parse channel index */
	arg = argv[1];
	if (!kp_parse_non_negative_number(arg, &idx) ||
			idx >= ARRAY_SIZE(kp_cap_ch_conf_list)) {
		shell_error(
			shell,
			"Invalid channel index (0-%zu expected): %s",
			ARRAY_SIZE(kp_cap_ch_conf_list) - 1,
			arg
		);
		return 1;
	}

	/* Output channel configuration */
	conf = &kp_cap_ch_conf_list[idx];
	shell_print(shell, "%s %s %s",
			kp_cap_dir_to_lcstr(conf->dir),
			(conf->rising ? "rising" : "falling"),
			conf->name);
	return 0;
}

/** Execute the "get timeout" command */
static int
kp_cmd_get_timeout(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(shell, "%u us", kp_cap_timeout_us);
	return 0;
}

/** Execute the "get bounce" command */
static int
kp_cmd_get_bounce(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(shell, "%u us", kp_cap_bounce_us);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(get_subcmds,
	SHELL_CMD(speed, NULL,
			"Get speed percentage",
			kp_cmd_get_speed),
	SHELL_CMD(top, NULL, "Restore the top position",
			kp_cmd_get_top),
	SHELL_CMD(bottom, NULL, "Restore the bottom position",
			kp_cmd_get_bottom),
	SHELL_CMD_ARG(ch, NULL,
			"Get channel configuration: "
			"<idx> -> none/up/down/both rising/falling <name>",
			kp_cmd_get_ch, 2, 0),
	SHELL_CMD(timeout, NULL,
			"Get capture timeout, us",
			kp_cmd_get_timeout),
	SHELL_CMD(bounce, NULL,
			"Get bounce time, us",
			kp_cmd_get_bounce),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(get, &get_subcmds,
			"Get parameters", NULL);

/** Sampling result code */
enum kp_sample_rc {
	/* Success */
	KP_SAMPLE_RC_OK,
	/* Aborted */
	KP_SAMPLE_RC_ABORTED,
	/* Actuator is off */
	KP_SAMPLE_RC_OFF,
};

/**
 * Sample captured channels for a specified movement.
 *
 * @param target	The absolute actuator position to move to.
 * 			If invalid, no movement will be done.
 * @param dir		The capture direction.
 * 			If KP_CAP_DIR_NONE, no capture will be done, or waited
 * 			for, nor any results will be output.
 * @param ch_res_list	Location for channel capture results.
 * 			Can be NULL, if ch_num is zero.
 * @param ch_num	Number of channels to capture (and to output into the
 *			ch_res_list location). Will be truncated to the number
 *			of channels in "kp_cap_ch_conf_list". If zero, no
 *			capture will be done or waited for, nor any results
 *			will be output.
 *
 * @return Result code.
 */
static enum kp_sample_rc
kp_sample(int32_t target, enum kp_cap_dir dir,
		struct kp_cap_ch_res *ch_res_list, size_t ch_num)
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
	bool moving = kp_act_pos_is_valid(target);
	bool capturing = ch_num > 0 && dir;
	bool moved = false;
	bool captured = false;
	enum kp_input_msg msg;
	size_t i;

	assert(ch_res_list != NULL || ch_num == 0);

	ch_num = MIN(ch_num, ARRAY_SIZE(kp_cap_ch_conf_list));

	/* Initialize events */
	kp_input_get_event_init(&events[EVENT_IDX_INPUT]);
	kp_act_finish_move_event_init(&events[EVENT_IDX_ACT_FINISH_MOVE]);
	kp_cap_finish_event_init(&events[EVENT_IDX_CAP_FINISH]);

	/* Start the capture, if requested */
	if (capturing) {
		kp_cap_start(kp_cap_ch_conf_list, ch_num, dir,
				kp_cap_timeout_us, kp_cap_bounce_us);
	} else {
		events[EVENT_IDX_CAP_FINISH].type = K_POLL_TYPE_IGNORE;
	}

	/* Start moving towards the target, if requested */
	if (moving) {
		kp_act_start_move_to(target, kp_act_speed);
	} else {
		events[EVENT_IDX_ACT_FINISH_MOVE].type = K_POLL_TYPE_IGNORE;
	}

	/* Move and capture */
	for (; (moving && !moved) || (capturing && !captured);) {
		while (k_poll(events, ARRAY_SIZE(events), K_FOREVER) != 0);

		/* Handle input */
		if (events[EVENT_IDX_INPUT].state) {
			while (kp_input_get(&msg, K_FOREVER) != 0);
			if (msg == KP_INPUT_MSG_ABORT) {
				if (moving) {
					kp_act_abort();
				}
				if (capturing) {
					kp_cap_abort();
				}
			}
		}

		/* Handle movement completion */
		if (events[EVENT_IDX_ACT_FINISH_MOVE].state) {
			move_rc = kp_act_finish_move(K_FOREVER);
			moved = true;
		}

		/* Handle capture completion */
		if (events[EVENT_IDX_CAP_FINISH].state) {
			cap_rc = kp_cap_finish(
				ch_res_list, ch_num, K_FOREVER);
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

/**
 * Count the number of all-channel triggers for a number of passes over a
 * range of actuator positions.
 *
 * @param top		The top position of the range.
 * @param bottom	The bottom position of the range.
 * @param passes	Number of actuator passes to execute.
 * @param ptriggers	Location for the number of triggered passes.
 * 			Can be NULL to have the number discarded.
 *
 * @return Sampling result code.
 */
static enum kp_sample_rc
kp_check(int32_t top, int32_t bottom, size_t passes, size_t *ptriggers)
{
	struct kp_cap_ch_res ch_res_list[ARRAY_SIZE(kp_cap_ch_conf_list)];
	enum kp_sample_rc rc = KP_CAP_RC_OK;
	int32_t pos;
	bool start_top;
	size_t pass;
	size_t captured_pass;
	size_t triggers = 0;
	size_t i;
	size_t enabled_channels;
	size_t captured_channels;
	size_t triggered_channels;

	assert(kp_act_pos_is_valid(top));
	assert(kp_act_pos_is_valid(bottom));

	/* Count enabled channels */
	for (i = 0, enabled_channels = 0;
		i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
		if (kp_cap_ch_conf_list[i].dir) {
			enabled_channels++;
		}
	}
	assert(enabled_channels > 0);

	if (passes == 0) {
		goto finish;
	}

	/* Move to the closest boundary without capturing */
	pos = kp_act_locate();
	if (!kp_act_pos_is_valid(pos)) {
		return KP_SAMPLE_RC_OFF;
	}
	start_top = abs(pos - kp_act_pos_top) < abs(pos - kp_act_pos_bottom);
	rc = kp_sample(start_top ? kp_act_pos_top : kp_act_pos_bottom,
		       KP_CAP_DIR_NONE, NULL, 0);
	if (rc != KP_SAMPLE_RC_OK) {
		return rc;
	}

	for (pass = 0, captured_pass = 0; captured_pass < passes; pass++) {
		bool at_top = (pass & 1) ^ start_top;
		/* Capture moving to the opposite boundary */
		rc = kp_sample(
			at_top ? bottom : top,
			at_top ? KP_CAP_DIR_DOWN : KP_CAP_DIR_UP,
			ch_res_list, ARRAY_SIZE(ch_res_list)
		);
		if (rc != KP_SAMPLE_RC_OK) {
			return rc;
		}

		/* Check if any and all channels triggered */
		for (i = 0, captured_channels = 0, triggered_channels = 0;
		     i < ARRAY_SIZE(ch_res_list); i++) {
			enum kp_cap_ch_status status = ch_res_list[i].status;
			if (status != KP_CAP_CH_STATUS_DISABLED) {
				captured_channels++;
				if (status != KP_CAP_CH_STATUS_TIMEOUT) {
					triggered_channels++;
				}
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

/** Execute the "check" command */
static int
kp_cmd_check(const struct shell *shell, size_t argc, char **argv)
{
	size_t i;
	size_t enabled_channels;
	int32_t start;
	long passes;
	size_t triggers;

	/* Check for power */
	if (kp_act_is_off()) {
		shell_error(shell, "Actuator is off, aborting");
		return 1;
	}
	/* Check for parameters */
	if (!kp_act_pos_is_valid(kp_act_pos_top)) {
		shell_error(shell, "Top position not set, aborting");
		return 1;
	}
	if (!kp_act_pos_is_valid(kp_act_pos_bottom)) {
		shell_error(shell, "Bottom position not set, aborting");
		return 1;
	}

	/* Count enabled channels */
	for (i = 0, enabled_channels = 0;
		i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
		if (kp_cap_ch_conf_list[i].dir) {
			enabled_channels++;
		}
	}

	/* Check that at least one channel is enabled */
	if (enabled_channels == 0) {
		shell_error(shell, "No enabled channels, aborting");
		shell_info(shell,
			   "Use \"set ch\" command to enable channels");
		return 1;
	}

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_check, kp_input_bypass_cb);
	kp_input_reset();

	if (argc < 2) {
		passes = 1;
	} else {
		if (!kp_parse_non_negative_number(argv[1], &passes) ||
				passes == 0) {
			shell_error(
				shell,
				"Invalid number of passes "
				"(a number greater than zero expected): %s",
				argv[1]
			);
			return 1;
		}
	}

	/* Remember the start position */
	start = kp_act_locate();

	/* Count the triggers */
	switch (kp_check(kp_act_pos_top, kp_act_pos_bottom,
				(size_t)passes, &triggers)) {
		case KP_SAMPLE_RC_OK:
			break;
		case KP_SAMPLE_RC_ABORTED:
			shell_error(shell, "Aborted");
			return 1;
		case KP_SAMPLE_RC_OFF:
			shell_error(shell, "Actuator is off, aborted");
			return 1;
		default:
			shell_error(shell, "Unexpected error, aborted");
			return 1;
	}

	shell_print(shell, "%zu%%", triggers * 100 / (size_t)passes);

	/* Return to the start position */
	switch (kp_act_move_to(start, kp_act_speed)) {
		case KP_ACT_MOVE_RC_OK:
			break;
		case KP_ACT_MOVE_RC_ABORTED:
			shell_warn(
				shell,
				"Move back to the start position "
				"was aborted"
			);
			break;
		case KP_ACT_MOVE_RC_OFF:
			shell_warn(
				shell,
				"Couldn't move back to the start position - "
				"actuator is off"
			);
			break;
		default:
			shell_error(
				shell,
				"Unexpected error moving back to the start "
				"position"
			);
			break;
	}

	return 0;
}

SHELL_CMD_ARG_REGISTER(check, NULL,
			"Check reliability of all-channel triggering "
			"between the top and bottom positions, over the "
			"specified number of passes (default is one)",
			kp_cmd_check, 1, 1);

/** Execute the "tighten" command */
static int
kp_cmd_tighten(const struct shell *shell, size_t argc, char **argv)
{
	const char *arg;
	long steps;
	long passes;
	size_t i;
	size_t enabled_channels;
	int32_t start;
	size_t triggers;
	int32_t top = KP_ACT_POS_INVALID;
	int32_t bottom = KP_ACT_POS_INVALID;
	int32_t middle = KP_ACT_POS_INVALID;
	int32_t next_top = kp_act_pos_top;
	int32_t next_bottom = kp_act_pos_bottom;
	int result = 1;

	/* Check for power */
	if (kp_act_is_off()) {
		shell_error(shell, "Actuator is off, aborting");
		return 1;
	}
	/* Check for parameters */
	if (!kp_act_pos_is_valid(kp_act_pos_top)) {
		shell_error(shell, "Top position not set, aborting");
		return 1;
	}
	if (!kp_act_pos_is_valid(kp_act_pos_bottom)) {
		shell_error(shell, "Bottom position not set, aborting");
		return 1;
	}

	/* Count enabled channels */
	for (i = 0, enabled_channels = 0;
		i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
		if (kp_cap_ch_conf_list[i].dir) {
			enabled_channels++;
		}
	}

	/* Check that at least one channel is enabled */
	if (enabled_channels == 0) {
		shell_error(shell, "No enabled channels, aborting");
		shell_info(shell,
			   "Use \"set ch\" command to enable channels");
		return 1;
	}

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_tighten, kp_input_bypass_cb);
	kp_input_reset();

	/* Parse the number of steps to tighten to */
	if (argc < 2) {
		steps = 1;
	} else {
		arg = argv[1];
		if (!kp_parse_non_negative_number(arg, &steps) ||
				steps == 0) {
			shell_error(
				shell,
				"Invalid number of steps to tighten to "
				"(a number greater than zero expected): %s",
				arg
			);
			return 1;
		}
	}

	/* Parse the number of passes to use for verifying */
	if (argc < 3) {
		passes = 2;
	} else {
		arg = argv[2];
		if (!kp_parse_non_negative_number(arg, &passes) ||
				passes == 0) {
			shell_error(
				shell,
				"Invalid number of passes "
				"(a number greater than zero expected): %s",
				arg
			);
			return 1;
		}
	}

	/* Remember the start position */
	start = kp_act_locate();

#define CHECK(_top, _bottom, _passes, _ptriggers) \
	do {                                                            \
		assert(_top < _bottom);                                 \
		assert(_passes > 0);                                    \
		switch (kp_check(_top, _bottom, _passes, _ptriggers)) { \
			case KP_SAMPLE_RC_OK:                           \
				break;                                  \
			case KP_SAMPLE_RC_ABORTED:                      \
				shell_error(shell, "Aborted");          \
				return 1;                               \
			case KP_SAMPLE_RC_OFF:                          \
				shell_error(shell,                      \
					"Actuator is off, aborted");    \
				return 1;                               \
			default:                                        \
				shell_error(shell,                      \
					"Unexpected error, aborted");   \
				return 1;                               \
		}                                                       \
	} while (0)

	while (true) {
		CHECK(next_top, next_bottom, (size_t)passes, &triggers);
		/* If we don't have reliable triggers */
		if (triggers < (size_t)passes) {
			/* If we tried the top half just now */
			if (next_bottom == middle) {
				/* Try the bottom half next */
				next_top = middle;
				next_bottom = bottom;
				continue;
			}
			/* Give up */
			break;
		}
		/* We have reliable triggers, accept the next range */
		top = next_top;
		bottom = next_bottom;
		/* If we can't reduce further by binary division */
		if ((bottom - top) < steps * 2) {
			break;
		}
		/* Try the top half next */
		middle = (top + bottom) / 2;
		next_top = top;
		next_bottom = middle;
	}

	/* While we're not fully tight yet */
	while (kp_act_pos_is_valid(top) && kp_act_pos_is_valid(bottom) &&
			(bottom - top) > steps &&
			(bottom - top) < steps * 2) {
		/* Try top-aligned range */
		next_top = top;
		next_bottom = top + steps;
		CHECK(next_top, next_bottom, (size_t)passes, &triggers);
		/* If that didn't work */
		if (triggers < (size_t)passes) {
			/* Try bottom-aligned range */
			next_top = bottom - steps;
			next_bottom = bottom;
			CHECK(next_top, next_bottom,
					(size_t)passes, &triggers);
			/* If even that didn't work */
			if (triggers < (size_t)passes) {
				/* Give up */
				break;
			}
		}
		top = next_top;
		bottom = next_bottom;
	}

#undef CHECK

	/* If we got a valid range */
	if (kp_act_pos_is_valid(top) && kp_act_pos_is_valid(bottom)) {
		if (bottom - top > steps) {
			shell_warn(shell,
				"Couldn't tighten to exactly %ld "
				"steps, stopped at %d",
				steps, bottom - top);
		}
		kp_act_pos_top = top;
		kp_act_pos_bottom = bottom;
		result = 0;
	} else {
		shell_error(shell,
			"No reliable trigger between the current "
			"top and bottom position, not narrowed"
		);
	}

	/* Return to the start position */
	switch (kp_act_move_to(start, kp_act_speed)) {
		case KP_ACT_MOVE_RC_OK:
			break;
		case KP_ACT_MOVE_RC_ABORTED:
			shell_warn(
				shell,
				"Move back to the start position "
				"was aborted"
			);
			break;
		case KP_ACT_MOVE_RC_OFF:
			shell_warn(
				shell,
				"Couldn't move back to the start position - "
				"actuator is off"
			);
			break;
		default:
			shell_warn(
				shell,
				"Unexpected error moving back to the start "
				"position"
			);
			break;
	}

	return result;
}

SHELL_CMD_ARG_REGISTER(tighten, NULL,
			"Move the top and bottom positions within the "
			"specified number of steps (default 1) around "
			"the trigger point. Verify trigger with "
			"specified number of passes (default 4).",
			kp_cmd_tighten, 1, 2);

/** Format string for the first column of "measure" command output */
static char kp_cmd_measure_col0_fmt[16] = {0, };
/** Format string for successive columns of "measure" command output */
static char kp_cmd_measure_coln_fmt[16] = {0, };
/** Width of the first column of "measure" command output */
#define KP_CMD_MEASURE_COL0_WIDTH	(KP_CAP_TIME_MAX_DIGITS + 1)
/** Width of successive columns of "measure" command output */
#define KP_CMD_MEASURE_COLN_WIDTH	KP_CAP_CH_NAME_MAX_LEN
/** Size of the column buffer for "measure" command output */
#define KP_CMD_MEASURE_COL_BUF_SIZE \
	(MAX(KP_CMD_MEASURE_COL0_WIDTH, KP_CMD_MEASURE_COLN_WIDTH + 1) + 1)

/** Initialize the global state of "measure" command output */
static void
kp_cmd_measure_output_setup(void)
{
	int rc;
	rc = snprintf(kp_cmd_measure_col0_fmt,
		      sizeof(kp_cmd_measure_col0_fmt),
		      "%%%zu.%zus",
		      KP_CMD_MEASURE_COL0_WIDTH, KP_CMD_MEASURE_COL0_WIDTH);
	assert(rc >= 0);
	assert(rc < sizeof(kp_cmd_measure_col0_fmt));
	rc = snprintf(kp_cmd_measure_coln_fmt,
		      sizeof(kp_cmd_measure_coln_fmt),
		      " %%%zu.%zus",
		      KP_CMD_MEASURE_COLN_WIDTH, KP_CMD_MEASURE_COLN_WIDTH);
	assert(rc >= 0);
	assert(rc < sizeof(kp_cmd_measure_coln_fmt));
}

/** The "measure" command output state */
struct kp_cmd_measure_output {
	/** The shell to output to */
	const struct shell *shell;
	/** The number of columns to output */
	size_t col_num;
	/** The index of the next column to output */
	size_t col_idx;
	/** The column formatting buffer */
	char col_buf[KP_CMD_MEASURE_COL_BUF_SIZE];
};

/**
 * Initializer for a measure output state
 *
 * @param _shell	The shell to output to.
 * @param _col_num	Number of columns to output.
 */
#define KP_CMD_MEASURE_OUTPUT_INIT(_shell, _col_num) \
	(struct kp_cmd_measure_output){     \
		.shell = _shell,            \
		.col_num = _col_num,        \
		.col_idx = 0,               \
		.col_buf = {0, },           \
	}

/**
 * Print a column to "measure" command output.
 *
 * @param out	The output to print to.
 * @param fmt	The format string to use to format the column.
 * @param ...	The arguments for the format string.
 */
static void
kp_cmd_measure_output_col(struct kp_cmd_measure_output *out,
			  const char *restrict fmt, ...)
{
	va_list args;
	int rc;

	assert(out != NULL);
	assert(out->col_idx < out->col_num);
	assert(fmt != NULL);

	va_start(args, fmt);
	rc = vsnprintf(out->col_buf, sizeof(out->col_buf), fmt, args);
	va_end(args);

	assert(rc >= 0);
	assert(rc < sizeof(out->col_buf));

	shell_fprintf(out->shell, SHELL_NORMAL,
		      (out->col_idx == 0 ? kp_cmd_measure_col0_fmt
				     : kp_cmd_measure_coln_fmt),
		      out->col_buf);
	out->col_idx++;
}

/**
 * Print a newline to "measure" command output.
 *
 * @param out	The output to print to.
 */
static void
kp_cmd_measure_output_nl(struct kp_cmd_measure_output *out)
{
	assert(out != NULL);
	assert(out->col_idx == 0 || out->col_idx == out->col_num);
	shell_fprintf(out->shell, SHELL_NORMAL, "\n");
	out->col_idx = 0;
}

/**
 * Print a separator line to "measure" command output.
 *
 * @param out		The output to print to.
 */
static void
kp_cmd_measure_output_sep(struct kp_cmd_measure_output *out)
{
	size_t i;

	assert(out != NULL);
	assert(out->col_idx == 0);

	memset(out->col_buf, '-', sizeof(out->col_buf) - 1);
	out->col_buf[sizeof(out->col_buf) - 1] = '\0';

	for (i = 0; i < out->col_num; i++) {
		shell_fprintf(out->shell, SHELL_NORMAL,
			      (i == 0 ? kp_cmd_measure_col0_fmt
				      : kp_cmd_measure_coln_fmt),
			      out->col_buf);
	}
	shell_fprintf(out->shell, SHELL_NORMAL, "\n");
	out->col_idx = 0;
}

#define COL(_out, _args...) kp_cmd_measure_output_col(_out, _args)
#define NL(_out) kp_cmd_measure_output_nl(_out)
#define SEP(_out) kp_cmd_measure_output_sep(_out)
#define DIR_NUM 3

/**
 * Print basic statistics for "measure" command output.
 *
 * @param out			The output to print to.
 * @param start_top		True if the first capture was going down.
 * 				False otherwise.
 * @param ch_res_list_list	A list (array) of channel result lists to
 *				summarize. Contains "passes" number of channel
 *				lists (arrays), each with KP_CAP_CH_NUM
 *				elements.
 * @param passes		Number of passes (channel result lists).
 * 				Must be greater than one.
 * @param verbose		True if the output should be verbose,
 * 				false otherwise.
 */
static void
kp_cmd_measure_output_stats(
		struct kp_cmd_measure_output *out,
		bool start_top,
		struct kp_cap_ch_res (*ch_res_list_list)[KP_CAP_CH_NUM],
		size_t passes,
		bool verbose)
{
	bool timeout[KP_CAP_CH_NUM][DIR_NUM] = {{0, }};
	bool overcapture[KP_CAP_CH_NUM][DIR_NUM] = {{0, }};
	bool unknown[KP_CAP_CH_NUM][DIR_NUM] = {{0, }};
	const char *metric_names[] = {
		"Trigs, %",
		"Min, us",
		"Max, us",
		"Mean, us"
	};
	const size_t metric_num = ARRAY_SIZE(metric_names);
	uint32_t metric_data[metric_num][KP_CAP_CH_NUM][DIR_NUM];
	/* NOTE: Code below expects triggers to occupy index zero */
	uint32_t (*triggers)[KP_CAP_CH_NUM][DIR_NUM] = &metric_data[0];
	uint32_t (*min)[KP_CAP_CH_NUM][DIR_NUM] = &metric_data[1];
	uint32_t (*max)[KP_CAP_CH_NUM][DIR_NUM] = &metric_data[2];
	uint32_t (*mean)[KP_CAP_CH_NUM][DIR_NUM] = &metric_data[3];
	/* Values found per channel per direction */
	bool got_value[KP_CAP_CH_NUM][DIR_NUM] = {{0, }, };
	size_t pass, ch, metric, dir;
	struct kp_cap_ch_res *ch_res;

	assert(out != NULL);
	assert((start_top & 1) == start_top);
	assert(ch_res_list_list != NULL);
	assert(passes > 1);

	/* Initialize metrics */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (dir = 0; dir < DIR_NUM; dir++) {
			(*triggers)[ch][dir] = 0;
			(*min)[ch][dir] = UINT32_MAX;
			(*max)[ch][dir] = 0;
		}
	}

	/* Find minimums and maximums */
	for (pass = 0; pass < passes; pass++) {
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			dir = (pass & 1) ^ start_top;
			ch_res = &ch_res_list_list[pass][ch];
			switch (ch_res->status) {
			case KP_CAP_CH_STATUS_DISABLED:
				break;
			case KP_CAP_CH_STATUS_TIMEOUT:
				/* Got timeout for this direction */
				timeout[ch][dir] = true;
				/* Got timeout for either direction */
				timeout[ch][2] = true;
				break;
			case KP_CAP_CH_STATUS_OVERCAPTURE:
				/* Got overcapture for this direction */
				overcapture[ch][dir] = true;
				/* Got overcapture for either direction */
				overcapture[ch][2] = true;
				/* FALLTHROUGH */
			case KP_CAP_CH_STATUS_OK:
#define ADJ_MIN(_lvalue) (_lvalue = MIN(_lvalue, ch_res->value_us))
#define ADJ_MAX(_lvalue) (_lvalue = MAX(_lvalue, ch_res->value_us))
				/* Got trigger/value for this direction */
				(*triggers)[ch][dir]++;
				got_value[ch][dir] = true;
				ADJ_MIN((*min)[ch][dir]);
				ADJ_MAX((*max)[ch][dir]);
				/* Got trigger/value for either direction */
				(*triggers)[ch][2]++;
				got_value[ch][2] = true;
				ADJ_MIN((*min)[ch][2]);
				ADJ_MAX((*max)[ch][2]);
#undef ADJ_MAX
#undef ADJ_MIN
				break;
			default:
				/* Got unknown status for this direction */
				unknown[ch][dir] = true;
				/* Got unknown status for either direction */
				unknown[ch][2] = true;
				break;
			}
		}
	}

	/* Convert trigger counters to percentage */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (dir = 0; dir < DIR_NUM; dir++) {
			/* Enabled counted direction */
			enum kp_cap_dir enabled_dir =
				(dir + 1) & kp_cap_ch_conf_list[ch].dir;
			/* If the channel is disabled in counted directions */
			if (!enabled_dir) {
				/* Assign invalid percentage */
				(*triggers)[ch][dir] = UINT32_MAX;
				continue;
			}
			(*triggers)[ch][dir] =
				(*triggers)[ch][dir] * 100 /
				/* If both counted directions are enabled */
				(enabled_dir == KP_CAP_DIR_BOTH
					/* Take percentage of all passes */
					? passes
					/*
					 * Else, take percentage of enabled
					 * direction's passes
					 */
					: ((passes >> 1) +
					   (passes &
					    ((enabled_dir - 1) ==
					     start_top))));
		}
	}

	/* Calculate means */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (dir = 0; dir < DIR_NUM; dir++) {
			(*mean)[ch][dir] =
				((*min)[ch][dir] + (*max)[ch][dir]) / 2;
		}
	}

	/*
	 * Output results per direction per metric per channel
	 */
	/* For each direction */
	for (dir = verbose ? 0 : 2; dir < DIR_NUM; dir++) {
		/* Output direction header */
		SEP(out);
		COL(out, "%s", kp_cap_dir_to_cpstr(dir + 1));
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			if (kp_cap_ch_conf_list[ch].dir) {
				COL(out, "Value");
			}
		}
		NL(out);
		SEP(out);
		/* For each metric */
		for (metric = 0; metric < metric_num; metric++) {
			/* Output metric name */
			COL(out, "%s", metric_names[metric]);
			/* For each channel */
			for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
				/*
				 * If the channel is disabled for this
				 * direction
				 */
				if (!(kp_cap_ch_conf_list[ch].dir &
				      (dir + 1))) {
					/* If the channel is enabled */
					if (kp_cap_ch_conf_list[ch].dir) {
						COL(out, "");
					}
					continue;
				}
				/* Output metric value and/or flags */
				COL(out,
				    /*
				     * If it's the trigger percentage
				     * (at metric index zero),
				     * or we have measured values
				     */
				    (!metric || got_value[ch][dir])
					    ? "%s%s%s%u" : "%s%s%s",
				    overcapture[ch][dir] ? "+" : "",
				    unknown[ch][dir] ? "?" : "",
				    timeout[ch][dir] ? "!" : "",
				    metric_data[metric][ch][dir]);
			}
			NL(out);
		}
	}
}

/**
 * Print a time histogram for "measure" command output.
 *
 * @param out			The output to print to.
 * @param start_top		True if the first capture was going down.
 * 				False otherwise.
 * @param ch_res_list_list	A list (array) of channel result lists to
 *				summarize. Contains "passes" number of channel
 *				lists (arrays), each with KP_CAP_CH_NUM
 *				elements.
 * @param passes		Number of passes (channel result lists).
 * 				Must be greater than zero.
 * @param verbose		True if the output should be verbose,
 * 				false otherwise.
 */
static void
kp_cmd_measure_output_histogram(
		struct kp_cmd_measure_output *out,
		bool start_top,
		struct kp_cap_ch_res (*ch_res_list_list)[KP_CAP_CH_NUM],
		size_t passes,
		bool verbose)
{
#define STEP_NUM 16
#define CHAR_NUM (KP_CMD_MEASURE_COLN_WIDTH - 1)
	uint32_t min, max;
	uint32_t step_size;
	size_t step_passes[KP_CAP_CH_NUM][DIR_NUM][STEP_NUM] = {{{0, }}};
	size_t max_step_passes[KP_CAP_CH_NUM][DIR_NUM] = {{0, }};
	size_t ch, pass, dir;
	struct kp_cap_ch_res *ch_res;
	ssize_t step_idx;
	uint32_t step_min;
	char char_buf[CHAR_NUM + 2] = {0, };
	size_t char_idx;
	size_t chars, next_chars;
	char c;

	assert(out != NULL);
	assert((start_top & 1) == start_top);
	assert(ch_res_list_list != NULL);
	assert(passes > 0);

	/* Find minimum and maximum time for all channels */
	min = UINT32_MAX;
	max = 0;
	for (pass = 0; pass < passes; pass++) {
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			ch_res = &ch_res_list_list[pass][ch];
			if (ch_res->status == KP_CAP_CH_STATUS_OK ||
			    ch_res->status ==
				KP_CAP_CH_STATUS_OVERCAPTURE) {
				min = MIN(min, ch_res->value_us);
				max = MAX(max, ch_res->value_us);
			}
		}
	}

	/* Calculate histogram step values per channel */
	step_size = (max - min) / STEP_NUM;
	if (step_size == 0) {
		step_size = KP_CAP_RES_US;
	}
	for (pass = 0; pass < passes; pass++) {
		dir = (pass & 1) ^ start_top;
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			ch_res = &ch_res_list_list[pass][ch];
			if (ch_res->status != KP_CAP_CH_STATUS_OK &&
			    ch_res->status != KP_CAP_CH_STATUS_OVERCAPTURE) {
				continue;
			}
			step_idx = MIN(
				(ch_res->value_us - min) / step_size,
				STEP_NUM - 1
			);
			/* Count this direction */
			step_passes[ch][dir][step_idx]++;
			/* Count both directions */
			step_passes[ch][2][step_idx]++;
		}
	}

	/* Calculate histogram maximums per channel per direction */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (dir = 0; dir < DIR_NUM; dir++) {
			for (step_idx = 0; step_idx < STEP_NUM; step_idx++) {
				max_step_passes[ch][dir] = MAX(
					max_step_passes[ch][dir],
					step_passes[ch][dir][step_idx]
				);
			}
		}
	}

	/* Scale histograms down to characters */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (dir = 0; dir < DIR_NUM; dir++) {
			for (step_idx = 0; step_idx < STEP_NUM; step_idx++) {
				if (max_step_passes[ch][dir] == 0) {
					continue;
				}
				step_passes[ch][dir][step_idx] =
					step_passes[ch][dir][step_idx] *
					CHAR_NUM / max_step_passes[ch][dir];
			}
		}
	}

	/* Output header */
	SEP(out);
	COL(out, "Time");
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		if (kp_cap_ch_conf_list[ch].dir) {
			COL(out, "Triggers");
		}
	}
	NL(out);

	/*
	 * Output histograms per direction
	 */
	/* For each direction */
	for (dir = verbose ? 0 : 2; dir < DIR_NUM; dir++) {
		/* Output direction header */
		SEP(out);
		COL(out, "%s, us", kp_cap_dir_to_cpstr(dir + 1));
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			/* If channel is not enabled in this direction */
			if (!(kp_cap_ch_conf_list[ch].dir & (dir + 1))) {
				/* If channel is enabled */
				if (kp_cap_ch_conf_list[ch].dir) {
					COL(out, "");
				}
				continue;
			}
			COL(out, "0%*zu", CHAR_NUM, max_step_passes[ch][dir]);
		}
		NL(out);
		/* For each line of histograms (step_num + 2) */
		for (step_idx = -1, step_min = min - step_size;
		     step_idx <= STEP_NUM;
		     step_idx++, step_min += step_size) {
			/* Output line header value */
			if (step_idx < 0) {
				COL(out, "");
			} else {
				COL(out, "%u", step_min);
			}
			/* Output histogram bars per channel */
			for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
				/* If channel is not enabled in this direction */
				if (!(kp_cap_ch_conf_list[ch].dir & (dir + 1))) {
					/* If channel is enabled */
					if (kp_cap_ch_conf_list[ch].dir) {
						COL(out, "");
					}
					continue;
				}
				chars = (step_idx >= 0 && step_idx < STEP_NUM)
					? step_passes[ch][dir][step_idx]
					: 0;
				next_chars = (step_idx < STEP_NUM - 1)
					? step_passes[ch][dir][step_idx + 1]
					: 0;
				/* For each character in the column buffer */
				for (char_idx = 0; char_idx <= CHAR_NUM;
				     char_idx++) {
					if (char_idx == 0) {
						c = '|';
					} else if (char_idx == chars) {
						c = '|';
					} else if (char_idx == CHAR_NUM) {
						c = ':';
					} else if (char_idx > chars) {
						if (char_idx < next_chars) {
							c = '_';
						} else {
							c = ' ';
						}
					} else if (char_idx < chars) {
						if (char_idx > next_chars) {
							c = '_';
						} else {
							c = ' ';
						}
					}
					char_buf[char_idx] = c;
				}
				COL(out, "%s", char_buf);
			}
			NL(out);
		}
	}

#undef CHAR_NUM
#undef STEP_NUM
}

/** Execute the "measure" command */
static int
kp_cmd_measure(const struct shell *shell, size_t argc, char **argv)
{
	const char *arg;
	long passes;
	bool verbose;
	size_t i;
	size_t enabled_ch_num;
	size_t named_ch_num;
	struct kp_cmd_measure_output out;
	static struct kp_cap_ch_res *ch_res;
	size_t pass;
	enum kp_sample_rc rc = KP_CAP_RC_OK;
	int32_t start_pos;
	/* True if we started at the top (and went down), false otherwise */
	bool start_top;

	/* Check for power */
	if (kp_act_is_off()) {
		shell_error(shell, "Actuator is off, aborting");
		return 1;
	}
	/* Check for parameters */
	if (!kp_act_pos_is_valid(kp_act_pos_top)) {
		shell_error(shell, "Top position not set, aborting");
		return 1;
	}
	if (!kp_act_pos_is_valid(kp_act_pos_bottom)) {
		shell_error(shell, "Bottom position not set, aborting");
		return 1;
	}

	/* Collect channel stats */
	for (i = 0, enabled_ch_num = 0, named_ch_num = 0;
		i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
		if (kp_cap_ch_conf_list[i].dir) {
			enabled_ch_num++;
		}
		if (kp_cap_ch_conf_list[i].name[0] != '\0') {
			named_ch_num++;
		}
	}
	/* Check that at least one channel is enabled */
	if (enabled_ch_num == 0) {
		shell_error(shell, "No enabled channels, aborting");
		shell_info(shell,
			   "Use \"set ch\" command to enable channels");
		return 1;
	}

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_measure, kp_input_bypass_cb);
	kp_input_reset();

	/* Parse the number of passes to measure */
	if (argc < 2) {
		passes = 1;
	} else {
		arg = argv[1];
		if (!kp_parse_non_negative_number(arg, &passes) ||
		    passes == 0 ||
		    passes > ARRAY_SIZE(kp_cap_ch_res_list_list)) {
			shell_error(
				shell,
				"Invalid number of passes "
				"(1-%zu expected): %s",
				ARRAY_SIZE(kp_cap_ch_res_list_list), arg
			);
			return 1;
		}
	}

	/* Parse the verbosity flag */
	if (argc < 3) {
		verbose = false;
	} else {
		arg = argv[2];
		if (kp_strcasecmp(arg, "verbose") == 0) {
			verbose = true;
		} else if (kp_strcasecmp(arg, "brief") == 0) {
			verbose = false;
		} else {
			shell_error(
				shell,
				"Invalid verbosity argument "
				"(brief/verbose expected): %s",
				arg
			);
			return 1;
		}
	}

	/* Remember the start position */
	start_pos = kp_act_locate();
	if (!kp_act_pos_is_valid(start_pos)) {
		rc = KP_SAMPLE_RC_OFF;
		goto finish;
	}

	/* Move to the closest boundary without capturing */
	start_top = abs(start_pos - kp_act_pos_top) <
		abs(start_pos - kp_act_pos_bottom);
	rc = kp_sample(start_top ? kp_act_pos_top : kp_act_pos_bottom,
		       KP_CAP_DIR_NONE, NULL, 0);
	if (rc != KP_SAMPLE_RC_OK) {
		goto finish;
	}

	/* Initialize the output */
	out = KP_CMD_MEASURE_OUTPUT_INIT(shell, enabled_ch_num + 1);

	/* Output the channel index header */
	COL(&out, "");
	for (i = 0; i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
		if (kp_cap_ch_conf_list[i].dir) {
			COL(&out, "#%zu", i);
		}
	}
	NL(&out);

	/* Output the channel name header, if any are named */
	if (named_ch_num != 0) {
		COL(&out, "");
		for (i = 0; i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
			if (kp_cap_ch_conf_list[i].dir) {
				COL(&out, "%s", kp_cap_ch_conf_list[i].name);
			}
		}
		NL(&out);
	}

	/* Output timing header, if verbose or doing one pass only */
	if (verbose || passes == 1) {
		SEP(&out);
		COL(&out, "Up/Down");
		for (i = 0; i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
			if (kp_cap_ch_conf_list[i].dir) {
				COL(&out, "Time, us");
			}
		}
		NL(&out);
		SEP(&out);
	}

	/* Capture the requested number of passes */
	for (pass = 0; pass < passes; pass++) {
		bool at_top = (pass & 1) ^ start_top;

		/* Capture moving to the opposite boundary */
		rc = kp_sample(
			at_top ? kp_act_pos_bottom : kp_act_pos_top,
			at_top ? KP_CAP_DIR_DOWN : KP_CAP_DIR_UP,
			kp_cap_ch_res_list_list[pass],
			ARRAY_SIZE(kp_cap_ch_res_list_list[pass])
		);
		if (rc != KP_SAMPLE_RC_OK) {
			goto finish;
		}

		/* Skip output if not verbose or doing one pass only */
		if (!(verbose || passes == 1)) {
			continue;
		}

		COL(&out, at_top ? "Down" : "Up");
		for (i = 0;
		     i < ARRAY_SIZE(kp_cap_ch_res_list_list[pass]);
		     i++) {
			if (!kp_cap_ch_conf_list[i].dir) {
				continue;
			}
			ch_res = &kp_cap_ch_res_list_list[pass][i];
			switch (ch_res->status) {
			case KP_CAP_CH_STATUS_DISABLED:
				COL(&out, "");
				break;
			case KP_CAP_CH_STATUS_TIMEOUT:
				COL(&out, "!");
				break;
			case KP_CAP_CH_STATUS_OVERCAPTURE:
				COL(&out, "+%u", ch_res->value_us);
				break;
			case KP_CAP_CH_STATUS_OK:
				COL(&out, "%u", ch_res->value_us);
				break;
			default:
				COL(&out, "?");
				break;
			}
		}
		NL(&out);
	}

	if (passes > 1) {
		kp_cmd_measure_output_stats(
			&out, start_top, kp_cap_ch_res_list_list,
			passes, verbose
		);
		kp_cmd_measure_output_histogram(
			&out, start_top, kp_cap_ch_res_list_list,
			passes, verbose
		);
	}

	SEP(&out);

finish:

	/* If we failed */
	if (rc != KP_SAMPLE_RC_OK) {
		return 1;
	}

	/* Try to return to the start position */
	switch (kp_act_move_to(start_pos, kp_act_speed)) {
		case KP_ACT_MOVE_RC_OK:
			break;
		case KP_ACT_MOVE_RC_ABORTED:
			shell_warn(
				shell,
				"Move back to the start position "
				"was aborted"
			);
			break;
		case KP_ACT_MOVE_RC_OFF:
			shell_warn(
				shell,
				"Couldn't move back to the start position - "
				"actuator is off"
			);
			break;
		default:
			shell_error(
				shell,
				"Unexpected error moving back to the start "
				"position"
			);
			break;
	}

	return 0;
}

#undef DIR_NUM
#undef SEP
#undef COL
#undef NL

SHELL_CMD_ARG_REGISTER(measure, NULL,
			"Measure timing on all enabled channels for "
			"specified number of passes (default 1), and output "
			"brief (default), or verbose results",
			kp_cmd_measure, 1, 2);

void
main(void)
{
	const struct device *dev;
	size_t i;

	/* Initialize the global state of "measure" command output */
	kp_cmd_measure_output_setup();

	/* Check shell UART is ready */
	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	if (!device_is_ready(dev)) {
		return;
	}

	/* Initialize the shell extensions */
	kp_shell_init();

	/*
	 * Initialize GPIO pins
	 */
	if (!device_is_ready(kp_act_gpio)) {
		return;
	}
	if (!device_is_ready(kp_dbg_gpio)) {
		return;
	}
	gpio_pin_configure(kp_dbg_gpio, kp_dbg_pin_update,
				GPIO_PUSH_PULL | GPIO_OUTPUT_LOW);

	/*
	 * Initialize the actuator
	 */
	kp_act_init(kp_act_gpio, /* disable */ 3, /* dir */ 8, /* step */ 9);

	/*
	 * Set default capture channel configuration
	 */
	for (i = 0; i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
		gpio_pin_configure(kp_dbg_gpio, 4 + i,
					GPIO_PUSH_PULL | GPIO_OUTPUT_LOW);
		kp_cap_ch_conf_list[i] = (struct kp_cap_ch_conf){
			.dir = KP_CAP_DIR_NONE,
			.rising = true,
			.name = {0},
			.dbg_gpio = kp_dbg_gpio,
			.dbg_pin = kp_dbg_pin_ch_base + i,
		};
	}

	/*
	 * Initialize the capturer
	 */
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	struct stm32_pclken pclken = {
		.bus = DT_CLOCKS_CELL(KP_TIMER_NODE, bus),
		.enr = DT_CLOCKS_CELL(KP_TIMER_NODE, bits)
	};
	if (!device_is_ready(clk)) {
		return;
	}
	if (clock_control_on(clk, (clock_control_subsys_t *)&pclken) < 0) {
		return;
	}
	IRQ_CONNECT(DT_IRQ_BY_NAME(KP_TIMER_NODE, trgcom, irq),
		    DT_IRQ_BY_NAME(KP_TIMER_NODE, trgcom, priority),
		    kp_cap_isr, NULL, 0);
	irq_enable(DT_IRQ_BY_NAME(KP_TIMER_NODE, trgcom, irq));
	IRQ_CONNECT(DT_IRQ_BY_NAME(KP_TIMER_NODE, up, irq),
		    DT_IRQ_BY_NAME(KP_TIMER_NODE, up, priority),
		    kp_cap_isr, NULL, 0);
	irq_enable(DT_IRQ_BY_NAME(KP_TIMER_NODE, up, irq));
	IRQ_CONNECT(DT_IRQ_BY_NAME(KP_TIMER_NODE, cc, irq),
		    DT_IRQ_BY_NAME(KP_TIMER_NODE, cc, priority),
		    kp_cap_isr, NULL, 0);
	irq_enable(DT_IRQ_BY_NAME(KP_TIMER_NODE, cc, irq));
	kp_cap_init((TIM_TypeDef *)DT_REG_ADDR(KP_TIMER_NODE),
		    kp_dbg_gpio, kp_dbg_pin_update);
}
