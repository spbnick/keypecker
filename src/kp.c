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
#include "kp_sample.h"
#include "kp_table.h"
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
			while (k_poll(events, ARRAY_SIZE(events),
				      K_FOREVER) != 0);

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

/** Capture configuration */
static struct kp_cap_conf kp_cap_conf;

/* List of channel capture results */
static struct kp_cap_ch_res kp_cap_ch_res_list[512 * KP_CAP_CH_NUM];

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
			idx >= ARRAY_SIZE(kp_cap_conf.ch_list)) {
		shell_error(
			shell,
			"Invalid channel index (0-%zu expected): %s",
			ARRAY_SIZE(kp_cap_conf.ch_list) - 1,
			arg
		);
		return 1;
	}

	/* Read current state */
	conf = kp_cap_conf.ch_list[idx];

	/* Parse capture directions, if specified */
	if (argc >= 3) {
		arg = argv[2];
		if (!kp_cap_dirs_from_str(arg, &conf.dirs)) {
			shell_error(
				shell,
				"Invalid capture directions "
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
	kp_cap_conf.ch_list[idx] = conf;

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
	if ((uint32_t)timeout_us + kp_cap_conf.bounce_us >=
	    KP_CAP_TIME_MAX_US) {
		shell_error(
			shell,
			"Timeout plus bounce time exceed "
			"maximum capture time: %ld + %u >= %u",
			timeout_us, kp_cap_conf.bounce_us, KP_CAP_TIME_MAX_US
		);
		return 1;
	};
	kp_cap_conf.timeout_us = (uint32_t)timeout_us;
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
	if (kp_cap_conf.timeout_us + (uint32_t)bounce_us >=
	    KP_CAP_TIME_MAX_US) {
		shell_error(
			shell,
			"Bounce time plus timeout exceed "
			"maximum capture time: %ld + %u >= %u",
			bounce_us, kp_cap_conf.timeout_us, KP_CAP_TIME_MAX_US
		);
		return 1;
	};
	kp_cap_conf.bounce_us = (uint32_t)bounce_us;
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
			idx >= ARRAY_SIZE(kp_cap_conf.ch_list)) {
		shell_error(
			shell,
			"Invalid channel index (0-%zu expected): %s",
			ARRAY_SIZE(kp_cap_conf.ch_list) - 1,
			arg
		);
		return 1;
	}

	/* Output channel configuration */
	conf = &kp_cap_conf.ch_list[idx];
	shell_print(shell, "%s %s %s",
			kp_cap_dirs_to_lcstr(conf->dirs),
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
	shell_print(shell, "%u us", kp_cap_conf.timeout_us);
	return 0;
}

/** Execute the "get bounce" command */
static int
kp_cmd_get_bounce(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(shell, "%u us", kp_cap_conf.bounce_us);
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

/** Execute the "check" command */
static int
kp_cmd_check(const struct shell *shell, size_t argc, char **argv)
{
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

	/* Check that at least one channel is enabled */
	if (kp_cap_conf_ch_num(&kp_cap_conf, KP_CAP_DIRS_BOTH) == 0) {
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
	switch (kp_sample_check(kp_act_pos_top, kp_act_pos_bottom,
				kp_act_speed, (size_t)passes,
				&kp_cap_conf, &triggers)) {
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

	/* Check that at least one channel is enabled */
	if (kp_cap_conf_ch_num(&kp_cap_conf, KP_CAP_DIRS_BOTH) == 0) {
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

#define CHECK(_top, _bottom, _ptriggers) \
	do {                                                            \
		assert((_top) < (_bottom));                             \
		switch (kp_sample_check(_top, _bottom, kp_act_speed,    \
					(size_t)passes, &kp_cap_conf,   \
					_ptriggers)) {                  \
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
		CHECK(next_top, next_bottom, &triggers);
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
		CHECK(next_top, next_bottom, &triggers);
		/* If that didn't work */
		if (triggers < (size_t)passes) {
			/* Try bottom-aligned range */
			next_top = bottom - steps;
			next_bottom = bottom;
			CHECK(next_top, next_bottom, &triggers);
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


/**
 * Print basic statistics for "measure" command output.
 *
 * @param table		The table output to print to.
 * @param conf		The configuration the capture was done with.
 * @param even_down	True if even passes are directed down, false if up.
 * @param ch_res_list	A list (array) of channel results to summarize.
 * 			Contains only results for channels enabled for either
 * 			direction in turn.
 * @param passes	Number of passes. Must be greater than one.
 * @param verbose	True if the output should be verbose,
 * 			false otherwise.
 */
static void
kp_cmd_measure_output_stats(
		struct kp_table *table,
		const struct kp_cap_conf *conf,
		bool even_down,
		struct kp_cap_ch_res *ch_res_list,
		size_t passes,
		bool verbose)
{
	bool timeout[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	bool overcapture[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	bool unknown[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	const char *metric_names[] = {
		"Trigs, %",
		"Min, us",
		"Max, us",
		"Mean, us"
	};
	const size_t metric_num = ARRAY_SIZE(metric_names);
	uint32_t metric_data[metric_num][KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM];
	/* NOTE: Code below expects triggers to occupy index zero */
	uint32_t (*triggers)[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] =
							&metric_data[0];
	uint32_t (*min)[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = &metric_data[1];
	uint32_t (*max)[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = &metric_data[2];
	uint32_t (*mean)[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = &metric_data[3];
	/* Values found per channel per direction set */
	bool got_value[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }, };
	size_t pass, ch, metric;
	enum kp_cap_ne_dirs ne_dirs;
	struct kp_cap_ch_res *ch_res;

	assert(table != NULL);
	assert(kp_cap_conf_is_valid(conf));
	assert((even_down & 1) == even_down);
	assert(ch_res_list != NULL);
	assert(passes > 1);

	/* Initialize metrics */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (ne_dirs = 0; ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
			(*triggers)[ch][ne_dirs] = 0;
			(*min)[ch][ne_dirs] = UINT32_MAX;
			(*max)[ch][ne_dirs] = 0;
		}
	}

	/* Find minimums and maximums */
	for (ch_res = ch_res_list, pass = 0; pass < passes; pass++) {
		ne_dirs = kp_cap_ne_dirs_from_down(even_down ^ (pass & 1));
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			/* Skip channels disabled in this direction */
			if (!(conf->ch_list[ch].dirs &
			      kp_cap_dirs_from_ne(ne_dirs))) {
				continue;
			}
			/* Aggregate the channel result */
			switch (ch_res->status) {
			case KP_CAP_CH_STATUS_TIMEOUT:
				/* Got timeout for this direction */
				timeout[ch][ne_dirs] = true;
				/* Got timeout for either direction */
				timeout[ch][KP_CAP_NE_DIRS_BOTH] = true;
				break;
			case KP_CAP_CH_STATUS_OVERCAPTURE:
				/* Got overcapture for this direction */
				overcapture[ch][ne_dirs] = true;
				/* Got overcapture for either direction */
				overcapture[ch][KP_CAP_NE_DIRS_BOTH] = true;
				/* FALLTHROUGH */
			case KP_CAP_CH_STATUS_OK:
#define ADJ_MIN(_lvalue) (_lvalue = MIN(_lvalue, ch_res->value_us))
#define ADJ_MAX(_lvalue) (_lvalue = MAX(_lvalue, ch_res->value_us))
				/* Got trigger/value for this direction */
				(*triggers)[ch][ne_dirs]++;
				got_value[ch][ne_dirs] = true;
				ADJ_MIN((*min)[ch][ne_dirs]);
				ADJ_MAX((*max)[ch][ne_dirs]);
				/* Got trigger/value for either direction */
				(*triggers)[ch][KP_CAP_NE_DIRS_BOTH]++;
				got_value[ch][KP_CAP_NE_DIRS_BOTH] = true;
				ADJ_MIN((*min)[ch][KP_CAP_NE_DIRS_BOTH]);
				ADJ_MAX((*max)[ch][KP_CAP_NE_DIRS_BOTH]);
#undef ADJ_MAX
#undef ADJ_MIN
				break;
			default:
				/* Got unknown status for this direction */
				unknown[ch][ne_dirs] = true;
				/* Got unknown status for either direction */
				unknown[ch][KP_CAP_NE_DIRS_BOTH] = true;
				break;
			}
			ch_res++;
		}
	}

	/* Convert trigger counters to percentage */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (ne_dirs = 0; ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
			/* Enabled counted direction */
			enum kp_cap_dirs enabled_dirs =
				kp_cap_dirs_from_ne(ne_dirs) &
				conf->ch_list[ch].dirs;
			/* If the channel is disabled in counted directions */
			if (!enabled_dirs) {
				/* Assign invalid percentage */
				(*triggers)[ch][ne_dirs] = UINT32_MAX;
				continue;
			}
			(*triggers)[ch][ne_dirs] =
				(*triggers)[ch][ne_dirs] * 100 /
				/* If both counted directions are enabled */
				(enabled_dirs == KP_CAP_DIRS_BOTH
					/* Take percentage of all passes */
					? passes
					/*
					 * Else, take percentage of enabled
					 * direction's passes
					 */
					: ((passes >> 1) +
					   (passes &
					    (enabled_dirs ==
					     kp_cap_dirs_from_down(
						even_down
					     )))));
		}
	}

	/* Calculate means */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (ne_dirs = 0; ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
			(*mean)[ch][ne_dirs] =
				((*min)[ch][ne_dirs] +
				 (*max)[ch][ne_dirs]) / 2;
		}
	}

	/*
	 * Output results per direction per metric per channel
	 */
	/* For each non-empty direction combination */
	for (ne_dirs = verbose ? 0 : KP_CAP_NE_DIRS_BOTH;
			ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
		/* Output direction header */
		kp_table_sep(table);
		kp_table_col(
			table, "%s",
			kp_cap_dirs_to_cpstr(kp_cap_dirs_from_ne(ne_dirs))
		);
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			if (conf->ch_list[ch].dirs) {
				kp_table_col(table, "Value");
			}
		}
		kp_table_nl(table);
		kp_table_sep(table);
		/* For each metric */
		for (metric = 0; metric < metric_num; metric++) {
			/* Output metric name */
			kp_table_col(table, "%s", metric_names[metric]);
			/* For each channel */
			for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
				/*
				 * If the channel is disabled for this
				 * direction
				 */
				if (!(conf->ch_list[ch].dirs &
				      kp_cap_dirs_from_ne(ne_dirs))) {
					/* If the channel is enabled */
					if (conf->ch_list[ch].dirs) {
						kp_table_col(table, "");
					}
					continue;
				}
				/* Output metric value and/or flags */
				kp_table_col(table,
				    /*
				     * If it's the trigger percentage
				     * (at metric index zero),
				     * or we have measured values
				     */
				    (!metric || got_value[ch][ne_dirs])
					    ? "%s%s%s%u" : "%s%s%s",
				    overcapture[ch][ne_dirs] ? "+" : "",
				    unknown[ch][ne_dirs] ? "?" : "",
				    timeout[ch][ne_dirs] ? "!" : "",
				    metric_data[metric][ch][ne_dirs]);
			}
			kp_table_nl(table);
		}
	}
}

/**
 * Print a time histogram for "measure" command output.
 *
 * @param table		The table output to print to.
 * @param conf		The configuration the capture was done with.
 * @param even_down	True if even passes are directed down, false if up.
 * @param ch_res_list	A list (array) of channel results to summarize.
 * 			Contains only results for channels enabled for either
 * 			direction in turn.
 * @param passes	Number of passes. Must be greater than zero.
 * @param verbose	True if the output should be verbose,
 * 			false otherwise.
 */
static void
kp_cmd_measure_output_histogram(
		struct kp_table *table,
		const struct kp_cap_conf *conf,
		bool even_down,
		struct kp_cap_ch_res *ch_res_list,
		size_t passes,
		bool verbose)
{
#define STEP_NUM 16
#define CHAR_NUM (KP_CAP_CH_NAME_MAX_LEN - 1)
	uint32_t min, max;
	uint32_t step_size;
	size_t step_passes[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM]
				[STEP_NUM] = {{{0, }}};
	size_t max_step_passes[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	size_t ch, pass;
	enum kp_cap_ne_dirs ne_dirs;
	struct kp_cap_ch_res *ch_res;
	ssize_t step_idx;
	uint32_t step_min;
	char char_buf[CHAR_NUM + 2] = {0, };
	size_t char_idx;
	size_t chars, next_chars;
	char c;

	assert(table != NULL);
	assert(kp_cap_conf_is_valid(conf));
	assert((even_down & 1) == even_down);
	assert(ch_res_list != NULL);
	assert(passes > 0);

	/* Find minimum and maximum time for all channels */
	min = UINT32_MAX;
	max = 0;
	for (ch_res = ch_res_list, pass = 0; pass < passes; pass++) {
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			if (conf->ch_list[ch].dirs &
			    kp_cap_dirs_from_down((pass & 1) ^ even_down)) {
				if (ch_res->status == KP_CAP_CH_STATUS_OK ||
				    ch_res->status ==
					KP_CAP_CH_STATUS_OVERCAPTURE) {
					min = MIN(min, ch_res->value_us);
					max = MAX(max, ch_res->value_us);
				}
				ch_res++;
			}
		}
	}

	/* Calculate histogram step values per channel */
	step_size = (max - min) / STEP_NUM;
	if (step_size == 0) {
		step_size = KP_CAP_RES_US;
	}
	for (ch_res = ch_res_list, pass = 0; pass < passes; pass++) {
		ne_dirs = kp_cap_ne_dirs_from_down(even_down ^ (pass & 1));
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			if (!(conf->ch_list[ch].dirs &
			      kp_cap_dirs_from_ne(ne_dirs))) {
			     continue;
			}
			if (ch_res->status == KP_CAP_CH_STATUS_OK ||
			    ch_res->status == KP_CAP_CH_STATUS_OVERCAPTURE) {
				step_idx = MIN(
					(ch_res->value_us - min) / step_size,
					STEP_NUM - 1
				);
				/* Count this direction */
				step_passes[ch][ne_dirs][step_idx]++;
				/* Count both directions */
				step_passes[ch]
					[KP_CAP_NE_DIRS_BOTH][step_idx]++;
			}
			ch_res++;
		}
	}

	/* Calculate histogram maximums per channel per direction */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (ne_dirs = 0; ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
			for (step_idx = 0; step_idx < STEP_NUM; step_idx++) {
				max_step_passes[ch][ne_dirs] = MAX(
					max_step_passes[ch][ne_dirs],
					step_passes[ch][ne_dirs][step_idx]
				);
			}
		}
	}

	/* Scale histograms down to characters */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (ne_dirs = 0; ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
			for (step_idx = 0; step_idx < STEP_NUM; step_idx++) {
				if (max_step_passes[ch][ne_dirs] == 0) {
					continue;
				}
				step_passes[ch][ne_dirs][step_idx] =
					step_passes[ch][ne_dirs][step_idx] *
					CHAR_NUM /
					max_step_passes[ch][ne_dirs];
			}
		}
	}

	/* Output header */
	kp_table_sep(table);
	kp_table_col(table, "Time");
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		if (conf->ch_list[ch].dirs) {
			kp_table_col(table, "Triggers");
		}
	}
	kp_table_nl(table);

	/*
	 * Output histograms per each (non-empty) combination of directions
	 */
	/* For each direction */
	for (ne_dirs = verbose ? 0 : KP_CAP_NE_DIRS_BOTH;
			ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
		/* Output direction header */
		kp_table_sep(table);
		kp_table_col(
			table, "%s, us",
			kp_cap_dirs_to_cpstr(kp_cap_dirs_from_ne(ne_dirs))
		);
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			/* If channel is not enabled in this direction */
			if (!(conf->ch_list[ch].dirs &
			      kp_cap_dirs_from_ne(ne_dirs))) {
				/* If channel is enabled for a direction */
				if (conf->ch_list[ch].dirs) {
					kp_table_col(table, "");
				}
				continue;
			}
			kp_table_col(table, "0%*zu",
				     CHAR_NUM, max_step_passes[ch][ne_dirs]);
		}
		kp_table_nl(table);
		/* For each line of histograms (step_num + 2) */
		for (step_idx = -1, step_min = min - step_size;
		     step_idx <= STEP_NUM;
		     step_idx++, step_min += step_size) {
			/* Output line header value */
			if (step_idx < 0) {
				kp_table_col(table, "");
			} else {
				kp_table_col(table, "%u", step_min);
			}
			/* Output histogram bars per channel */
			for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
				/* If channel direction is not enabled */
				if (!(conf->ch_list[ch].dirs &
				      kp_cap_dirs_from_ne(ne_dirs))) {
					/* If channel is enabled */
					if (conf->ch_list[ch].dirs) {
						kp_table_col(table, "");
					}
					continue;
				}
				chars = (step_idx >= 0 && step_idx < STEP_NUM)
					? step_passes[ch][ne_dirs][step_idx]
					: 0;
				next_chars = (step_idx < STEP_NUM - 1)
					? step_passes[ch][ne_dirs]
							[step_idx + 1]
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
				kp_table_col(table, "%s", char_buf);
			}
			kp_table_nl(table);
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
	struct kp_table table;
	static struct kp_cap_ch_res *ch_res;
	size_t ch_res_rem;
	size_t pass, captured_passes;
	enum kp_sample_rc rc = KP_CAP_RC_OK;
	int32_t start_pos;
	/* True if even passes are directed down, false if up */
	bool even_down;
	/* Directions that we'll capture in */
	enum kp_cap_dirs dirs;

	/* Check for power and remember the start position */
	if (!kp_act_pos_is_valid(start_pos = kp_act_locate())) {
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
	/* Decide on the initial direction */
	even_down = abs(start_pos - kp_act_pos_top) <
		abs(start_pos - kp_act_pos_bottom);

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_measure, kp_input_bypass_cb);
	kp_input_reset();

	/* Parse the number of passes to measure */
	if (argc < 2) {
		passes = 1;
	} else {
		arg = argv[1];
		if (!kp_parse_non_negative_number(arg, &passes)) {
			shell_error(
				shell,
				"Invalid number of passes "
				"(non-negative integer expected): %s",
				arg
			);
			return 1;
		}
	}

	/* Calculate the directions we'll capture in */
	dirs = (passes == 0)
		? KP_CAP_DIRS_NONE
		: ((passes == 1)
			? kp_cap_dirs_from_down(even_down)
			: KP_CAP_DIRS_BOTH);

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
	/* Force verbose, if doing one pass only */
	verbose = verbose || (passes == 1);

	/* Collect channel stats */
	for (i = 0, enabled_ch_num = 0, named_ch_num = 0;
		i < ARRAY_SIZE(kp_cap_conf.ch_list); i++) {
		if (kp_cap_conf.ch_list[i].dirs & dirs) {
			enabled_ch_num++;
			if (kp_cap_conf.ch_list[i].name[0] != '\0') {
				named_ch_num++;
			}
		}
	}

	/* Check that at least one channel is enabled */
	if (enabled_ch_num == 0) {
		shell_error(shell, "No enabled channels, aborting");
		shell_info(shell,
			   "Use \"set ch\" command to enable channels");
		return 1;
	}

	/* Check that we have enough memory to record all passes */
	i = kp_cap_conf_ch_res_idx(&kp_cap_conf, even_down, passes, 0);
	if (i >= ARRAY_SIZE(kp_cap_ch_res_list)) {
		shell_error(
			shell,
			"Not enough memory to capture measurement results.\n"
			"Available: %zu, required: %zu.\n",
			ARRAY_SIZE(kp_cap_ch_res_list), i
		);
		return 1;
	}

	/* Move to the closest boundary without capturing */
	rc = kp_sample(even_down ? kp_act_pos_top : kp_act_pos_bottom,
		       kp_act_speed, NULL, KP_CAP_DIRS_NONE, NULL, 0);
	if (rc != KP_SAMPLE_RC_OK) {
		goto finish;
	}

	/* Initialize the table output */
	kp_table_init(&table, shell,
		      KP_CAP_TIME_MAX_DIGITS + 1,
		      KP_CAP_CH_NAME_MAX_LEN,
		      enabled_ch_num + 1);

	/* Output the channel index header */
	kp_table_col(&table, "");
	for (i = 0; i < ARRAY_SIZE(kp_cap_conf.ch_list); i++) {
		if (kp_cap_conf.ch_list[i].dirs & dirs) {
			kp_table_col(&table, "#%zu", i);
		}
	}
	kp_table_nl(&table);

	/* Output the channel name header, if any are named */
	if (named_ch_num != 0) {
		kp_table_col(&table, "");
		for (i = 0; i < ARRAY_SIZE(kp_cap_conf.ch_list); i++) {
			if (kp_cap_conf.ch_list[i].dirs & dirs) {
				kp_table_col(&table, "%s",
					     kp_cap_conf.ch_list[i].name);
			}
		}
		kp_table_nl(&table);
	}

	/* Output timing header, if verbose or doing one pass only */
	if (verbose || passes == 1) {
		kp_table_sep(&table);
		kp_table_col(&table, "Up/Down");
		for (i = 0; i < ARRAY_SIZE(kp_cap_conf.ch_list); i++) {
			if (kp_cap_conf.ch_list[i].dirs & dirs) {
				kp_table_col(&table, "Time, us");
			}
		}
		kp_table_nl(&table);
		kp_table_sep(&table);
	}

	/* Capture the requested number of passes */
	for (ch_res_rem = ARRAY_SIZE(kp_cap_ch_res_list),
	     ch_res = kp_cap_ch_res_list,
	     captured_passes = 0,
	     pass = 0; pass < passes; pass++) {
		enum kp_cap_dirs dir =
			kp_cap_dirs_from_down((pass ^ even_down) & 1);
		bool captured = false;

		/* Capture moving to the opposite boundary */
		rc = kp_sample(
			dir == KP_CAP_DIRS_DOWN ? kp_act_pos_bottom
						: kp_act_pos_top,
			kp_act_speed, &kp_cap_conf, dir, ch_res, ch_res_rem
		);
		if (rc != KP_SAMPLE_RC_OK) {
			goto finish;
		}

		/* Output direction in the first column, if verbose */
		if (verbose) {
			kp_table_col(&table, "%s", kp_cap_dirs_to_cpstr(dir));
		}

		/* Count/output channel results */
		for (i = 0; i < ARRAY_SIZE(kp_cap_conf.ch_list); i++) {
			/* Skip channels disabled for this run */
			if (!(kp_cap_conf.ch_list[i].dirs & dirs)) {
				continue;
			}
			/* If channel is disabled in this direction only */
			if (!(kp_cap_conf.ch_list[i].dirs & dir)) {
				/* Output blank column, if verbose */
				if (verbose) {
					kp_table_col(&table, "");
				}
				/* Skip it */
				continue;
			}
			/* Output channel result, if verbose */
			if (verbose) {
				switch (ch_res->status) {
				case KP_CAP_CH_STATUS_TIMEOUT:
					kp_table_col(&table, "!");
					break;
				case KP_CAP_CH_STATUS_OVERCAPTURE:
					kp_table_col(&table, "+%u",
						     ch_res->value_us);
					break;
				case KP_CAP_CH_STATUS_OK:
					kp_table_col(&table, "%u",
						     ch_res->value_us);
					break;
				default:
					kp_table_col(&table, "?");
					break;
				}
			}
			/* Move onto next channel result */
			ch_res++;
			ch_res_rem--;
			captured = true;
		}

		/* Finish the line, if verbose */
		if (verbose) {
			kp_table_nl(&table);
		}

		captured_passes += captured;
	}

	if (captured_passes > 1) {
		kp_cmd_measure_output_stats(
			&table, &kp_cap_conf, even_down, kp_cap_ch_res_list,
			passes, verbose
		);
		kp_cmd_measure_output_histogram(
			&table, &kp_cap_conf, even_down, kp_cap_ch_res_list,
			passes, verbose
		);
	}

	kp_table_sep(&table);

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
	 * Set default capture configuration
	 */
	kp_cap_conf.timeout_us = 1000000;
	kp_cap_conf.bounce_us = 50000;
	for (i = 0; i < ARRAY_SIZE(kp_cap_conf.ch_list); i++) {
		kp_cap_conf.ch_list[i] = (struct kp_cap_ch_conf){
			.dirs = KP_CAP_DIRS_NONE,
			.rising = true,
			.name = {0},
		};
	}

	/*
	 * Configure capturer debug output
	 */
	struct kp_cap_dbg_conf cap_dbg_conf = {
		.gpio = kp_dbg_gpio,
		.update_pin = kp_dbg_pin_update,
	};
	for (i = 0; i < ARRAY_SIZE(cap_dbg_conf.cap_pin_list); i++) {
		gpio_pin_configure(cap_dbg_conf.gpio, kp_dbg_pin_ch_base + i,
					GPIO_PUSH_PULL | GPIO_OUTPUT_LOW);
		cap_dbg_conf.cap_pin_list[i] = kp_dbg_pin_ch_base + i;
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
	kp_cap_init((TIM_TypeDef *)DT_REG_ADDR(KP_TIMER_NODE), &cap_dbg_conf);
}
