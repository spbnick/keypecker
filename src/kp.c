/** @file
 *  @brief Keypecker main entry point
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_act.h"
#include "kp_shell.h"
#include <assert.h>
#include <stdlib.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/shell/shell.h>

/* Devicetree node identifier for the GPIO port */
#define KP_GPIO_NODE DT_NODELABEL(gpiob)

/* The GPIO port device */
static const struct device *kp_gpio = DEVICE_DT_GET(KP_GPIO_NODE);

/* The trigger input GPIO pin */
static const gpio_pin_t KP_GPIO_PIN_TRIGGER = 4;

/** Current actuator power */
static kp_act_power kp_act_power_curr = KP_ACT_POWER_OFF;

/** Top actuator position */
static kp_act_pos kp_pos_top = KP_ACT_POS_INVALID;

/** Bottom actuator position */
static kp_act_pos kp_pos_bottom = KP_ACT_POS_INVALID;

/** Execute the "on" command */
static int
kp_cmd_on(const struct shell *shell, size_t argc, char **argv)
{
	kp_act_power power;
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	power = kp_act_on();
	if (kp_act_power_is_valid(power)) {
		kp_act_power_curr = power;
		return 0;
	}
	shell_error(shell, "Actuator is already on");
	return 1;
}

SHELL_CMD_REGISTER(on, NULL, "Turn on actuator", kp_cmd_on);

/** Execute the "off" command */
static int
kp_cmd_off(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (kp_act_off(kp_act_power_curr)) {
		kp_pos_top = KP_ACT_POS_INVALID;
		kp_pos_bottom = KP_ACT_POS_INVALID;
		kp_act_power_curr = KP_ACT_POWER_OFF;
		return 0;
	}
	shell_error(shell, "Actuator is already off");
	return 1;
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
	rc = kp_act_move_by(kp_act_power_curr, -steps);
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
	rc = kp_act_move_by(kp_act_power_curr, steps);
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

/** True if the currently-running interactive command was aborted */
static volatile bool kp_cmd_aborted;

/**
 * Process input from the bypassed shell of a scheduled command.
 *
 * @param shell Shell instance.
 * @param data  Raw data from transport.
 * @param len   Data length.
 */
static void
kp_shell_input(const struct shell *shell, uint8_t *data, size_t len)
{
	for (; len > 0; data++, len--) {
		/* If Ctrl-C is pressed */
		if (*data == 0x03) {
			kp_cmd_aborted = true;
			kp_act_abort(kp_act_power_curr);
			shell_set_bypass(shell, NULL);
			break;
		}
	}
}

/** Execute the "swing steps" command */
static int
kp_cmd_swing(const struct shell *shell, size_t argc, char **argv)
{
	long steps;
	enum kp_act_move_rc rc;

	/* Return to the shell and restart in an input-diverted thread */
	kp_cmd_aborted = false;
	KP_SHELL_YIELD(kp_cmd_swing, kp_shell_input);

	/* Parse arguments */
	assert(argc == 2);
	if (!kp_parse_non_negative_number(argv[1], &steps) || steps == 0) {
		shell_error(shell, "Invalid number of steps: %s",
				argv[1]);
		return 1;
	}

	/* Move */
	rc = kp_act_move_by(kp_act_power_curr, steps / 2);
	while (rc == KP_ACT_MOVE_RC_OK && !kp_cmd_aborted) {
		steps = -steps;
		rc = kp_act_move_by(kp_act_power_curr, steps);
	}
	if (kp_cmd_aborted) {
		rc = KP_ACT_MOVE_RC_ABORTED;
	}
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

/** Execute the "set top" command */
static int
kp_cmd_set_top(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	kp_pos_top = kp_act_locate();
	if (kp_act_pos_is_valid(kp_pos_top)) {
		return 0;
	}
	shell_error(shell, "Actuator is off, position not set");
	return 1;
}

/** Execute the "set bottom" command */
static int
kp_cmd_set_bottom(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	kp_pos_bottom = kp_act_locate();
	if (kp_act_pos_is_valid(kp_pos_bottom)) {
		return 0;
	}
	shell_error(shell, "Actuator is off, position not set");
	return 1;
}

SHELL_STATIC_SUBCMD_SET_CREATE(set_subcmds,
	SHELL_CMD(top, NULL, "Register current position as the top",
			kp_cmd_set_top),
	SHELL_CMD(bottom, NULL, "Register current position as the bottom",
			kp_cmd_set_bottom),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(set, &set_subcmds,
			"Register current actuator position", NULL);

void
main(void)
{
	const struct device *dev;
	uint32_t dtr = 0;

	/* Initialize shell extensions */
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
	 * Initialize shell UART
	 */
	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	if (!device_is_ready(dev) || usb_enable(NULL)) {
		return;
	}

	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}
}
