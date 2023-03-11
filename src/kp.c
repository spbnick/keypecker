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
#include "kp_meas.h"
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

/** Last measurement */
struct kp_meas kp_meas = KP_MEAS_INVALID;

/** Execute the "measure" command */
static int
kp_cmd_measure(const struct shell *shell, size_t argc, char **argv)
{
	const char *arg;
	long passes;
	bool verbose;
	size_t i;
	enum kp_sample_rc rc;
	int32_t start_pos;
	/* True if even passes are directed down, false if up */
	bool even_down;

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

	/* Check that at least one channel is enabled */
	if (kp_cap_conf_ch_num(&kp_cap_conf, KP_CAP_DIRS_BOTH) == 0) {
		shell_error(shell, "No enabled channels, aborting");
		shell_info(shell,
			   "Use \"set ch\" command to enable channels");
		return 1;
	}

	/* Check that we have enough memory to record all passes */
	i = kp_cap_conf_ch_res_idx(&kp_cap_conf, even_down, passes, 0);
	if (i > ARRAY_SIZE(kp_meas.ch_res_list)) {
		shell_error(
			shell,
			"Not enough memory to capture measurement results.\n"
			"Available: %zu, required: %zu.\n",
			ARRAY_SIZE(kp_meas.ch_res_list), i
		);
		return 1;
	}

	/* Initialize the measurement */
	kp_meas_init(&kp_meas, 
		     kp_act_pos_top, kp_act_pos_bottom,
		     kp_act_speed, passes,
		     &kp_cap_conf, even_down);
	/* Acquire and output the measurement */
	rc = kp_meas_make(shell, &kp_meas, verbose);
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
				"Moving back to the start position "
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
