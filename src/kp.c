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
#include <zephyr/kernel.h>
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

/** The mutex protecting all the input state */
static K_MUTEX_DEFINE(kp_input_mutex);

/** Shell input messages */
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

/** Shell input message queue */
K_MSGQ_DEFINE(kp_input_msgq, sizeof(enum kp_input_msg),
		16, sizeof(enum kp_input_msg));

/** Shell input state */
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

/** Current shell input state */
static enum kp_input_st kp_input_st;

/**
 * Reset tracked shell input state to start processing another session.
 */
static void
kp_input_rset(void)
{
	k_mutex_lock(&kp_input_mutex, K_FOREVER);
	kp_input_st = KP_INPUT_ST_NONE;
	k_msgq_purge(&kp_input_msgq);
	k_mutex_unlock(&kp_input_mutex);
}

/**
 * Process input from the bypassed shell of a scheduled command.
 *
 * @param shell Shell instance.
 * @param data  Raw data from transport.
 * @param len   Data length.
 */
static void
kp_input_recv(const struct shell *shell, uint8_t *data, size_t len)
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
				kp_act_abort(kp_act_power_curr);
				shell_set_bypass(shell, NULL);
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

/** Execute the "swing steps" command */
static int
kp_cmd_swing(const struct shell *shell, size_t argc, char **argv)
{
	long steps;
	enum kp_act_move_rc rc;
	enum kp_input_msg msg;
	kp_act_pos start_pos;

	/* Parse arguments */
	assert(argc == 2 || argc == (size_t)SSIZE_MAX + 2);
	if (!kp_parse_non_negative_number(argv[1], &steps) || steps == 0) {
		shell_error(shell, "Invalid number of steps: %s",
				argv[1]);
		return 1;
	}

	/* Check for power */
	if (!kp_act_power_is_on(kp_act_power_curr)) {
		shell_error(shell, "Actuator is off, aborting");
		return 1;
	}

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_swing, kp_input_recv);
	kp_input_rset();

	/* Remember the start position */
	start_pos = kp_act_locate();

	/* Move */
	shell_print(shell, "Swinging, press Enter to stop, Ctrl-C to abort");
	rc = kp_act_move_by(kp_act_power_curr, steps / 2);
	while (rc == KP_ACT_MOVE_RC_OK) {
		if (!k_msgq_get(&kp_input_msgq, &msg, K_NO_WAIT)) {
			if (msg == KP_INPUT_MSG_ABORT) {
				rc = KP_ACT_MOVE_RC_ABORTED;
			} else if (msg == KP_INPUT_MSG_ENTER) {
				/* Return to the start position */
				rc = kp_act_move_to(start_pos);
				/* And stop */
				break;
			}
		} else {
			steps = -steps;
			rc = kp_act_move_by(kp_act_power_curr, steps);
		}
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
	if (!kp_act_power_is_on(kp_act_power_curr)) {
		shell_error(shell, "Actuator is off, aborting");
		return 1;
	}

	/* Return to the shell and restart in an input-diverted thread */
	KP_SHELL_YIELD(kp_cmd_adjust, kp_input_recv);
	kp_input_rset();

	/* Move */
	shell_print(shell,
		    "Press up and down arrow keys to move the actuator.");
	shell_print(shell, "Press Enter to stop, Ctrl-C to abort.");
	do {
		/* Read next input message */
		while (k_msgq_get(&kp_input_msgq, &msg, K_FOREVER) != 0);
		if (msg == KP_INPUT_MSG_UP || msg == KP_INPUT_MSG_DOWN) {
			rc = kp_act_move_by(
				kp_act_power_curr,
				(msg == KP_INPUT_MSG_DOWN) ? 1 : -1
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
