/** @file
 *  @brief Keypecker table output formatting
 */

/*
 * Copyright (c) 2023 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_table.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

void
kp_table_init(struct kp_table *table, const struct shell *shell,
	      size_t col0_width, size_t coln_width, size_t col_num)
{
	int rc;

	assert(table != NULL);
	assert(shell != NULL);
	assert(col0_width <= KP_TABLE_COL_WIDTH_MAX);
	assert(coln_width <= KP_TABLE_COL_WIDTH_MAX);

	memset(table, 0, sizeof(*table));

	table->shell = shell;

	rc = snprintf(table->col0_fmt, sizeof(table->col0_fmt),
		      "%%%zu.%zus", col0_width, col0_width);
	assert(rc >= 0);
	assert(rc < sizeof(table->col0_fmt));

	rc = snprintf(table->coln_fmt, sizeof(table->coln_fmt),
		      " %%%zu.%zus", coln_width, coln_width);
	assert(rc >= 0);
	assert(rc < sizeof(table->coln_fmt));

	table->col_num = col_num;
}

void
kp_table_col(struct kp_table *table, const char *restrict fmt, ...)
{
	va_list args;
	int rc;

	assert(kp_table_is_valid(table));
	assert(table->col_idx < table->col_num);
	assert(fmt != NULL);

	va_start(args, fmt);
	rc = vsnprintf(table->col_buf, sizeof(table->col_buf), fmt, args);
	va_end(args);

	assert(rc >= 0);
	assert(rc < sizeof(table->col_buf));

	shell_fprintf(table->shell, SHELL_NORMAL,
		      (table->col_idx == 0 ? table->col0_fmt
					   : table->coln_fmt),
		      table->col_buf);
	table->col_idx++;
}

void
kp_table_nl(struct kp_table *table)
{
	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0 || table->col_idx == table->col_num);
	shell_fprintf(table->shell, SHELL_NORMAL, "\n");
	table->col_idx = 0;
}

void
kp_table_sep(struct kp_table *table)
{
	size_t i;

	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0);

	memset(table->col_buf, '-', sizeof(table->col_buf) - 1);
	table->col_buf[sizeof(table->col_buf) - 1] = '\0';

	for (i = 0; i < table->col_num; i++) {
		shell_fprintf(table->shell, SHELL_NORMAL,
			      (i == 0 ? table->col0_fmt : table->coln_fmt),
			      table->col_buf);
	}
	shell_fprintf(table->shell, SHELL_NORMAL, "\n");
	table->col_idx = 0;
}
