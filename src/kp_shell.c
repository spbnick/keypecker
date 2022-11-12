/** @file
 *  @brief Keypecker shell extensions
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_shell.h"

/** The semaphore signaling a shell command can be scheduled */
static K_SEM_DEFINE(kp_shell_available, 1, 1);

/** The scheduled command's handler */
static shell_cmd_handler kp_shell_scheduled_handler;

/** The scheduled command's shell */
static const struct shell *kp_shell_scheduled_shell;

/** The scheduled command's argc */
static size_t kp_shell_scheduled_argc;

/** The scheduled command's argv buffer */
static char *kp_shell_scheduled_argv[KP_SHELL_ARGC_MAX];

/** The scheduled command's argument value buffer */
static char kp_shell_scheduled_argv_buf[KP_SHELL_ARGV_MAX];

/** The semaphore signaling a shell command is scheduled */
static K_SEM_DEFINE(kp_shell_scheduled, 0, 1);

/**
 * The command handler-executing thread function.
 */
static void
kp_shell_thread_fn(void *p1, void *p2, void *p3)
{
	/* Forever */
	while (true) {
		/* Wait for a command to be scheduled for execution */
		k_sem_take(&kp_shell_scheduled, K_FOREVER);

		/* Execute the command handler */
		kp_shell_scheduled_handler(
			kp_shell_scheduled_shell,
			/* Signal a scheduled invocation */
			kp_shell_scheduled_argc + (size_t)SSIZE_MAX,
			kp_shell_scheduled_argv
		);

		/*
		 * Dirty-dirty trick to be able to remove the bypass outside
		 * the callback and produce a relatively-normal prompt again.
		 */
		if (kp_shell_scheduled_shell->ctx->bypass /* Dirty! */) {
			shell_set_bypass(kp_shell_scheduled_shell, NULL);
			shell_stop(kp_shell_scheduled_shell);
			shell_start(kp_shell_scheduled_shell);
		}

		/* Signal another command can be scheduled */
		k_sem_give(&kp_shell_available);
	}
}

/** The thread executing scheduled command handlers */
K_THREAD_DEFINE(kp_shell_thread, 1024,
		kp_shell_thread_fn, NULL, NULL, NULL,
		-1, 0, -1);

bool
kp_shell_schedule(shell_cmd_handler handler,
		  const struct shell *shell, size_t argc, char **argv,
		  shell_bypass_cb_t bypass)
{
	char *argv_ptr;
	size_t argv_rem;
	size_t argi;
	size_t len;

	assert(handler != NULL);
	assert(shell != NULL);
	assert(argv != NULL);
	assert(bypass != NULL);

	/* If there are too many arguments */
	if (argc > (size_t)KP_SHELL_ARGC_MAX) {
		/* Signal the command didn't fit the buffer */
		return false;
	}

	/* Wait for the last-scheduled command to finish executing */
	k_sem_take(&kp_shell_available, K_FOREVER);

	/* Remember the command and its arguments */
	kp_shell_scheduled_handler = handler;
	kp_shell_scheduled_shell = shell;
	kp_shell_scheduled_argc = argc;
	for (argi = 0,
	     argv_ptr = kp_shell_scheduled_argv_buf,
	     argv_rem = sizeof(kp_shell_scheduled_argv_buf);
	     argi < argc;
	     argi++,
	     argv_ptr += len,
	     argv_rem -= len) {
		len = strlen(argv[argi]) + 1;
		/* If the argument wouldn't fit the buffer */
		if (len > argv_rem) {
			/* Mark the command schedule available again */
			k_sem_give(&kp_shell_available);
			/* Signal the command didn't fit the buffer */
			return false;
		}
		memcpy(argv_ptr, argv[argi], len);
		kp_shell_scheduled_argv[argi] = argv_ptr;
	}

	/* Install the input bypass */
	shell_set_bypass(shell, bypass);

	/* Signal to the thread a command is ready for execution */
	k_sem_give(&kp_shell_scheduled);

	/* Signal to the caller the command is scheduled */
	return true;
}

void
kp_shell_init(void)
{
	k_thread_start(kp_shell_thread);
}
