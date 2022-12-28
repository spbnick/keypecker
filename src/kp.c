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
#include <stm32_ll_tim.h>
#include <assert.h>
#include <stdlib.h>
#include <strings.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/shell/shell.h>

/* Devicetree node identifier for the GPIO port */
#define KP_GPIO_NODE DT_NODELABEL(gpiob)

/* Devicetree node identifier for the timer */
#define KP_TIMER_NODE DT_NODELABEL(timers1)

/* The GPIO port device */
static const struct device *kp_gpio = DEVICE_DT_GET(KP_GPIO_NODE);

/* The trigger input GPIO pin */
static const gpio_pin_t KP_GPIO_PIN_TRIGGER = 4;

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

/** Capture timeout, us */
static uint32_t kp_cap_timeout_us = 1000000;

/** Bounce time, us */
static uint32_t kp_cap_bounce_us = 50000;

static inline int
kp_strcasecmp(const char *a, const char *b)
{
	return strncasecmp(a, b, MAX(strlen(a), strlen(b)));
}

/** Execute the "set ch <idx> on/off [rising/falling [<name>]]" command */
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

	/* Parse capture status, if specified */
	if (argc >= 3) {
		arg = argv[2];
		if (kp_strcasecmp(arg, "on") == 0) {
			conf.capture = true;
		} else if (kp_strcasecmp(arg, "off") == 0) {
			conf.capture = false;
		} else {
			shell_error(
				shell,
				"Invalid capture status "
				"(on/off expected): %s",
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
			"<idx> on/off [rising/falling [<name>]]",
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
			(conf->capture ? "on" : "off"),
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
			"<idx> -> on/off rising/falling <name>",
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

/** Execute the "measure" command */
static int
kp_cmd_measure(const struct shell *shell, size_t argc, char **argv)
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
	struct kp_cap_ch_res ch_res_list[ARRAY_SIZE(kp_cap_ch_conf_list)];
	int32_t start_pos;
	bool moved = false;
	bool captured = false;
	enum kp_input_msg msg;

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

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_measure, kp_input_bypass_cb);
	kp_input_reset();

	/* Remember the start position */
	start_pos = kp_act_locate();

	/* Move to the top */
	move_rc = kp_act_move_to(kp_act_pos_top, kp_act_speed);
	if (move_rc != KP_ACT_MOVE_RC_OK) {
		goto finish;
	}

	/* Initialize events */
	kp_input_get_event_init(&events[EVENT_IDX_INPUT]);
	kp_act_finish_move_event_init(&events[EVENT_IDX_ACT_FINISH_MOVE]);
	kp_cap_finish_event_init(&events[EVENT_IDX_CAP_FINISH]);

	/* Start the capture */
	kp_cap_start(kp_cap_ch_conf_list, ARRAY_SIZE(kp_cap_ch_conf_list),
			kp_cap_timeout_us, kp_cap_bounce_us);
	/* Start moving towards the bottom */
	kp_act_start_move_to(kp_act_pos_bottom, kp_act_speed);

	while (!(moved && captured)) {
		while (k_poll(events, ARRAY_SIZE(events), K_FOREVER) != 0);

		if (events[EVENT_IDX_INPUT].state) {
			while (kp_input_get(&msg, K_FOREVER) != 0);
			if (msg == KP_INPUT_MSG_ABORT) {
				kp_act_abort();
				kp_cap_abort();
			}
		}

		if (events[EVENT_IDX_ACT_FINISH_MOVE].state) {
			move_rc = kp_act_finish_move(K_FOREVER);
			moved = true;
		}

		if (events[EVENT_IDX_CAP_FINISH].state) {
			cap_rc = kp_cap_finish(
				ch_res_list, ARRAY_SIZE(ch_res_list),
				K_FOREVER
			);
			captured = true;
		}

		/* Reset event state */
		for (size_t i = 0; i < ARRAY_SIZE(events); i++) {
			events[i].state = K_POLL_STATE_NOT_READY;
		}
	}

finish:

	if (move_rc == KP_ACT_MOVE_RC_OFF) {
		shell_error(shell, "Actuator is off, aborted");
	} else if (cap_rc == KP_CAP_RC_ABORTED ||
			move_rc == KP_ACT_MOVE_RC_ABORTED) {
		shell_error(shell, "Aborted");
	}

	if (move_rc != KP_ACT_MOVE_RC_OK || cap_rc != KP_CAP_RC_OK) {
		return 1;
	}

	/* Output results */
	for (size_t i = 0; i < ARRAY_SIZE(ch_res_list); i++) {
		struct kp_cap_ch_conf *ch_conf =
			&kp_cap_ch_conf_list[i];
		struct kp_cap_ch_res *ch_res = &ch_res_list[i];
		const char *status_str =
			kp_cap_ch_status_to_str(ch_res->status);
		switch (ch_res->status) {
		case KP_CAP_CH_STATUS_DISABLED:
			break;
		case KP_CAP_CH_STATUS_OK:
		case KP_CAP_CH_STATUS_OVERCAPTURE:
			shell_print(
				shell, "#%zu: %-15s %-11s %7u us",
				i, ch_conf->name, status_str,
				ch_res->value_us
			);
			break;
		default:
			shell_print(
				shell, "#%zu: %-15s %-11s",
				i, ch_conf->name, status_str
			);
			break;
		}
	}

	/* Return to the start position */
	move_rc = kp_act_move_to(start_pos, kp_act_speed);

	return move_rc != KP_ACT_MOVE_RC_OK;
}

SHELL_CMD_ARG_REGISTER(measure, NULL,
			"Move actuator between top and bottom positions, "
			"capturing the timing on all enabled channels",
			kp_cmd_measure, 1, 0);

#define TIMER_NODE
void
main(void)
{
	const struct device *dev;
	uint32_t dtr = 0;
	size_t i;

	/* Initialize USB */
	if (usb_enable(NULL)) {
		return;
	}

	/* Check shell UART is ready */
	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	if (!device_is_ready(dev)) {
		return;
	}

	/*
	 * Wait for DTR on shell UART
	 */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	/* Initialize the shell extensions */
	kp_shell_init();

	/*
	 * Initialize GPIO pins
	 */
	if (!device_is_ready(kp_gpio)) {
		return;
	}
	gpio_pin_configure(kp_gpio, KP_GPIO_PIN_TRIGGER, GPIO_INPUT);

	/*
	 * Initialize the actuator
	 */
	kp_act_init(kp_gpio, /* disable */ 3, /* dir */ 8, /* step */ 9);

	/*
	 * Set default capture channel configuration
	 */
	for (i = 0; i < ARRAY_SIZE(kp_cap_ch_conf_list); i++) {
		kp_cap_ch_conf_list[i] = (struct kp_cap_ch_conf){
			.capture = false,
			.rising = true,
			.name = {0},
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
	IRQ_CONNECT(DT_IRQ_BY_NAME(KP_TIMER_NODE, up, irq),
		    DT_IRQ_BY_NAME(KP_TIMER_NODE, up, priority),
		    kp_cap_isr, NULL, 0);
	irq_enable(DT_IRQ_BY_NAME(KP_TIMER_NODE, up, irq));
	IRQ_CONNECT(DT_IRQ_BY_NAME(KP_TIMER_NODE, cc, irq),
		    DT_IRQ_BY_NAME(KP_TIMER_NODE, cc, priority),
		    kp_cap_isr, NULL, 0);
	irq_enable(DT_IRQ_BY_NAME(KP_TIMER_NODE, cc, irq));
	kp_cap_init((TIM_TypeDef *)DT_REG_ADDR(KP_TIMER_NODE));
}
