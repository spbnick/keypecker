/** @file
 *  @brief Keypecker shell extensions
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_SHELL_H_
#define KP_SHELL_H_

#include <zephyr/shell/shell.h>
#include <stdbool.h>
#include <limits.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the shell extension module.
 */
extern void kp_shell_init(void);

/** Maximum number of arguments a scheduled command can be provided */
#define KP_SHELL_ARGC_MAX	16

#if (KP_SHELL_ARGC_MAX >= SSIZE_MAX)
#error "KP_SHELL_ARGC_MAX is too large"
#endif

/**
 * Maximum total length of argument strings a scheduled command can be
 * provided (including each string's terminating zero).
 */
#define KP_SHELL_ARGV_MAX	256

/**
 * Yield control back to the shell, with input bypassed, scheduling
 * re-executing the enclosing shell command handler in a separate thread, with
 * the same arguments.
 *
 * Expects the following symbols to be in scope, representing the handler
 * arguments:
 *
 *	const struct shell *shell;
 *	size_t argc;
 *	char **argv;
 *
 * If the handler arguments indicate it's invoked from the separate thread
 * (argc being greater than or equal to SSIZE_MAX), restore original arguments
 * and do nothing else.
 *
 * Must be placed at the start of the command handler function.
 *
 * The command won't be scheduled and an error message will be printed, if the
 * command has more than KP_SHELL_ARGC_MAX arguments, or all arguments
 * together take more memory than KP_SHELL_ARGV_MAX (including their
 * terminating zeroes).
 *
 * @param _handler	The command handler function invoking the macro.
 * 			The macro will verify that the function name matches
 * 			the enclosing function name.
 * @param _bypass	The shell "bypass" function to call when shell
 *			receives input. Must uninstall itself from the shell
 *			when input is no longer required.
 */
#define KP_SHELL_YIELD(_handler, _bypass) \
	do {                                                                \
		/* I wish the compiler would just give us current func */   \
		assert(strcmp(#_handler, __func__) == 0);                   \
		if (argc >= (size_t)SSIZE_MAX) {                            \
			argc -= (size_t)SSIZE_MAX;                          \
		} else {                                                    \
			if (kp_shell_schedule(_handler, shell, argc, argv,  \
					      _bypass)) {                   \
				return 0;                                   \
			}                                                   \
			shell_error(shell,                                  \
				    "Command too long, not executed");      \
			return 1;                                           \
		}                                                           \
	} while (0)

/**
 * Schedule an execution of a shell command handler with specified arguments
 * in a separate thread, and shell input bypassed. The handler would receive
 * argc set to the specified argc plus SSIZE_MAX to allow distinguishing such
 * invocation from a call by the shell itself.
 *
 * @param handler	The shell command handler to schedule execution of.
 * @param shell		The shell argument for the handler.
 * @param argc		The argc argument to the handler.
 * 			Must be less than SSIZE_MAX.
 * 			Must be less than KP_SHELL_ARGC_MAX.
 * @param argv		The argv argument for the handler.
 * 			Must point to strings occupying KP_SHELL_ARGV_MAX
 * 			bytes in total (including terminating zeros).
 * @param bypass	The shell "bypass" function receiving shell input.
 * 			Must uninstall itself from the shell when input is no
 * 			longer required.
 *
 * @return True if the command was scheduled succesfully.
 * 	   False if the command was too long and wasn't scheduled.
 */
extern bool kp_shell_schedule(shell_cmd_handler handler,
			      const struct shell *shell,
			      size_t argc, char **argv,
			      shell_bypass_cb_t bypass);

#ifdef __cplusplus
}
#endif

#endif /* KP_SHELL_H_ */
