/** @file
 *  @brief Keypecker table output formatting
 */

/*
 * Copyright (c) 2023 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_TABLE_H_
#define KP_TABLE_H_

#include <zephyr/shell/shell.h>
#include <zephyr/toolchain.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum width of a table column, characters */
#define KP_TABLE_COL_WIDTH_MAX	15

/** The table output state */
struct kp_table {
	/** The shell to output to */
	const struct shell *shell;
	/** Format string for the first column */
	char col0_fmt[16];
	/** Format string for successive columns */
	char coln_fmt[16];
	/** The number of columns to output */
	size_t col_num;
	/** The index of the next column to output */
	size_t col_idx;
	/** The column formatting buffer */
	char col_buf[KP_TABLE_COL_WIDTH_MAX + 1];
};

/**
 * Initialize a table output.
 *
 * @param table		The table output to initialize.
 * @param shell		The shell to output to.
 * @param col0_width	Width of the first column.
 *			Cannot be higher than KP_TABLE_COL_WIDTH_MAX.
 * @param coln_width	Width of the successive columns.
 *			Cannot be higher than KP_TABLE_COL_WIDTH_MAX.
 * @param col_num	Number of columns to output.
 */
extern void kp_table_init(struct kp_table *table, const struct shell *shell,
			  size_t col0_width, size_t coln_width,
			  size_t col_num);

/**
 * Print a column to table output.
 *
 * @param table	The table output to print to.
 * @param fmt	The format string to use to format the column.
 * @param ...	The arguments for the format string.
 */
extern void __printf_like(2, 3) kp_table_col(struct kp_table *table,
					     const char *restrict fmt, ...);

/**
 * Print a newline to table output.
 *
 * @param table	The table output to print to.
 */
extern void kp_table_nl(struct kp_table *table);

/**
 * Print a separator line to table output.
 *
 * @param table	The table output to print to.
 */
extern void kp_table_sep(struct kp_table *table);

#ifdef __cplusplus
}
#endif

#endif /* KP_TABLE_H_ */
