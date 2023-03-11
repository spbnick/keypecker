/** @file
 *  @brief Keypecker measurement
 */

/*
 * Copyright (c) 2023 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_meas.h"
#include "kp_table.h"
#include <sys/types.h>

enum kp_sample_rc
kp_meas_acquire(struct kp_meas *meas,
		kp_meas_acquire_pass_fn pass_fn,
		void *pass_data)
{
	enum kp_sample_rc rc;
	static struct kp_cap_ch_res *ch_res;
	size_t ch_res_rem;
	size_t ch_res_num;

	assert(kp_meas_is_valid(meas));
	assert(kp_meas_is_empty(meas));

	/* Move to the start boundary without capturing */
	rc = kp_sample(meas->even_down ? meas->top : meas->bottom,
		       meas->speed, &meas->conf, KP_CAP_DIRS_NONE, NULL, 0);
	if (rc != KP_SAMPLE_RC_OK) {
		return rc;
	}

	/* Capture the requested number of passes */
	for (ch_res_rem = ARRAY_SIZE(meas->ch_res_list),
	     ch_res = meas->ch_res_list;
	     meas->passes < meas->requested_passes;) {
		bool down = (meas->passes ^ meas->even_down) & 1;
		enum kp_cap_dirs dir = kp_cap_dirs_from_down(down);
		/* Count next number of channel results */
		ch_res_num = kp_cap_conf_ch_num(&meas->conf, dir);
		if (ch_res_num > ch_res_rem) {
			assert(!"No memory for results");
			/* The caller must make sure we have enough memory */
			return KP_SAMPLE_RC_OK;
		}
		/* Capture moving to the opposite boundary */
		rc = kp_sample(
			down ? meas->bottom : meas->top,
			meas->speed, &meas->conf, dir, ch_res, ch_res_rem
		);
		if (rc != KP_SAMPLE_RC_OK) {
			return rc;
		}
		/* Advance past captured results */
		ch_res_rem -= ch_res_num;
		ch_res += ch_res_num;
		/* Register the pass */
		meas->captured_passes += (ch_res_num != 0);
	       	meas->passes++;
		/* Notify about the pass, if requested */
		if (pass_fn != NULL) {
			pass_fn(meas, pass_data);
		}
	}

	return KP_SAMPLE_RC_OK;
}


/**
 * Output a channel index (and name) header for a measurement result.
 *
 * @param table		The table to output to.
 * @param meas		The measurement result to output the header for.
 */
static void
kp_meas_print_head(struct kp_table *table, const struct kp_meas *meas)
{
	enum kp_cap_dirs dirs;
	size_t i, named_ch_num;

	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0);
	assert(kp_meas_is_valid(meas));

	/* Get the directions we are capturing in */
	dirs = kp_meas_get_requested_dirs(meas);

	/* Collect channel stats */
	for (i = 0, named_ch_num = 0;
	     i < ARRAY_SIZE(meas->conf.ch_list); i++) {
		if (meas->conf.ch_list[i].dirs & dirs) {
			if (meas->conf.ch_list[i].name[0] != '\0') {
				named_ch_num++;
			}
		}
	}

	/* Output the channel index header */
	kp_table_col(table, "");
	for (i = 0; i < ARRAY_SIZE(meas->conf.ch_list); i++) {
		if (meas->conf.ch_list[i].dirs & dirs) {
			kp_table_col(table, "#%zu", i);
		}
	}
	kp_table_nl(table);

	/* Output the channel name header, if any are named */
	if (named_ch_num != 0) {
		kp_table_col(table, "");
		for (i = 0; i < ARRAY_SIZE(meas->conf.ch_list); i++) {
			if (meas->conf.ch_list[i].dirs & dirs) {
				kp_table_col(table, "%s",
					     meas->conf.ch_list[i].name);
			}
		}
		kp_table_nl(table);
	}
}

/**
 * Output a header for raw data of a measurement result.
 *
 * @param table		The table to output to.
 * @param meas		The measurement result to output.
 */
static void
kp_meas_print_data_head(struct kp_table *table, const struct kp_meas *meas)
{
	enum kp_cap_dirs dirs;
	size_t i;

	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0);
	assert(kp_meas_is_valid(meas));

	/* Get the directions we are capturing in */
	dirs = kp_meas_get_requested_dirs(meas);

	/* Output the header */
	kp_table_col(table, "Up/Down");
	for (i = 0; i < ARRAY_SIZE(meas->conf.ch_list); i++) {
		if (meas->conf.ch_list[i].dirs & dirs) {
			kp_table_col(table, "Time, us");
		}
	}
	kp_table_nl(table);
	kp_table_sep(table);
}

/**
 * Output a pass from raw data of a measurement result.
 *
 * @param table		The table to output to.
 * @param meas		The measurement result to output a pass from.
 * @param pass		The index of the pass to output.
 */
static void
kp_meas_print_data_pass(struct kp_table *table, const struct kp_meas *meas,
			size_t pass)
{
	enum kp_cap_dirs dirs;
	enum kp_cap_dirs pass_dir;
	size_t ch;
	const struct kp_cap_ch_res *ch_res;

	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0);
	assert(kp_meas_is_valid(meas));
	assert(pass < meas->passes);

	/* Get the directions we are capturing in */
	dirs = kp_meas_get_requested_dirs(meas);

	/* Get the direction of this pass */
	pass_dir = kp_cap_dirs_from_down((pass ^ meas->even_down) & 1);

	/* Do not output anything, if this pass has no data */
	if (kp_cap_conf_ch_num(&meas->conf, pass_dir) == 0) {
		return;
	}

	/* Output pass direction in the first column */
	kp_table_col(table, "%s", kp_cap_dirs_to_cpstr(pass_dir));

	for (ch = 0,
	     /* We promise we won't change the measurement */
	     ch_res = kp_meas_get_ch_res((struct kp_meas *)meas, pass, ch);
	     ch < ARRAY_SIZE(meas->conf.ch_list); ch++) {
		/* Skip channels disabled for this measurement */
		if (!(meas->conf.ch_list[ch].dirs & dirs)) {
			continue;
		}
		/* If channel is disabled in this direction only */
		if (!(meas->conf.ch_list[ch].dirs & pass_dir)) {
			/* Output blank column */
			kp_table_col(table, "");
			/* Skip it */
			continue;
		}
		/* Output channel result */
		switch (ch_res->status) {
		case KP_CAP_CH_STATUS_TIMEOUT:
			kp_table_col(table, "!");
			break;
		case KP_CAP_CH_STATUS_OVERCAPTURE:
			kp_table_col(table, "+%u",
				     ch_res->value_us);
			break;
		case KP_CAP_CH_STATUS_OK:
			kp_table_col(table, "%u",
				     ch_res->value_us);
			break;
		default:
			kp_table_col(table, "?");
			break;
		}
		/* Move onto the next channel result */
		ch_res++;
	}

	/* Finish the line */
	kp_table_nl(table);
}

/**
 * Output raw data of a measurement result.
 *
 * @param table		The table to output to.
 * @param meas		The measurement result to output.
 */
static void
kp_meas_print_data(struct kp_table *table, const struct kp_meas *meas)
{
	size_t pass;

	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0);
	assert(kp_meas_is_valid(meas));

	/* Output nothing, if got no data */
	if (meas->captured_passes == 0) {
		return;
	}

	/* Output the header */
	kp_meas_print_data_head(table, meas);

	/* Output each pass so far */
	for (pass = 0; pass < meas->passes; pass++) {
		kp_meas_print_data_pass(table, meas, pass);
	}
}

/**
 * Output basic statistics for a measurement result.
 *
 * @param table		The table to output to.
 * @param meas		The measurement result to output.
 * @param verbose	True if the output should be verbose,
 * 			false otherwise.
 */
static void
kp_meas_print_stats(struct kp_table *table,
		    const struct kp_meas *meas,
		    bool verbose)
{
	bool timeout[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	bool overcapture[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	bool unknown[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	static const char *metric_names[] = {
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
	const struct kp_cap_ch_res *ch_res;

	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0);
	assert(kp_meas_is_valid(meas));

	/* Initialize metrics */
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		for (ne_dirs = 0; ne_dirs < KP_CAP_NE_DIRS_NUM; ne_dirs++) {
			(*triggers)[ch][ne_dirs] = 0;
			(*min)[ch][ne_dirs] = UINT32_MAX;
			(*max)[ch][ne_dirs] = 0;
		}
	}

	/* Find minimums and maximums */
	for (ch_res = meas->ch_res_list, pass = 0;
	     pass < meas->passes; pass++) {
		ne_dirs = kp_cap_ne_dirs_from_down(
			meas->even_down ^ (pass & 1)
		);
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			/* Skip channels disabled in this direction */
			if (!(meas->conf.ch_list[ch].dirs &
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
				meas->conf.ch_list[ch].dirs;
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
					? meas->passes
					/*
					 * Else, take percentage of enabled
					 * direction's passes
					 */
					: ((meas->passes >> 1) +
					   (meas->passes &
					    (enabled_dirs ==
					     kp_cap_dirs_from_down(
						meas->even_down
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
			if (meas->conf.ch_list[ch].dirs) {
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
				if (!(meas->conf.ch_list[ch].dirs &
				      kp_cap_dirs_from_ne(ne_dirs))) {
					/* If the channel is enabled */
					if (meas->conf.ch_list[ch].dirs) {
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
 * Output histograms for a measurement result.
 *
 * @param table		The table to output to.
 * @param meas		The measurement result to output.
 * @param verbose	True if the output should be verbose,
 * 			false otherwise.
 */
static void
kp_meas_print_histogram(struct kp_table *table,
			const struct kp_meas *meas,
			bool verbose)
{
#define STEP_NUM 16
	uint32_t min, max;
	uint32_t step_size;
	size_t step_passes[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM]
				[STEP_NUM] = {{{0, }}};
	size_t max_step_passes[KP_CAP_CH_NUM][KP_CAP_NE_DIRS_NUM] = {{0, }};
	size_t ch, pass;
	enum kp_cap_dirs dirs;
	enum kp_cap_ne_dirs ne_dirs;
	const struct kp_cap_ch_res *ch_res;
	ssize_t step_idx;
	uint32_t step_min;
	char char_buf[sizeof(table->col_buf)];
	size_t width;
	size_t char_idx;
	size_t chars, next_chars;
	char c;

	assert(kp_table_is_valid(table));
	assert(table->col_idx == 0);
	assert(kp_meas_is_valid(meas));

	/* Get the directions we captured in */
	dirs = kp_meas_get_requested_dirs(meas);

	/* Set histogram width to one character less than column width */
	snprintf(char_buf, sizeof(char_buf), table->coln_fmt, "");
	width = strlen(char_buf);
	assert(width > 0);
	width--;

	/* Find minimum and maximum time for all channels */
	min = UINT32_MAX;
	max = 0;
	for (ch_res = meas->ch_res_list, pass = 0;
	     pass < meas->passes; pass++) {
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			if (meas->conf.ch_list[ch].dirs &
			    kp_cap_dirs_from_down((pass & 1) ^
						  meas->even_down)) {
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
	for (ch_res = meas->ch_res_list, pass = 0;
	     pass < meas->passes; pass++) {
		ne_dirs = kp_cap_ne_dirs_from_down(
			meas->even_down ^ (pass & 1)
		);
		for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
			if (!(meas->conf.ch_list[ch].dirs &
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
					width /
					max_step_passes[ch][ne_dirs];
			}
		}
	}

	/* Output header */
	kp_table_sep(table);
	kp_table_col(table, "Time");
	for (ch = 0; ch < KP_CAP_CH_NUM; ch++) {
		if (meas->conf.ch_list[ch].dirs & dirs) {
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
			if (!(meas->conf.ch_list[ch].dirs & dirs &
			      kp_cap_dirs_from_ne(ne_dirs))) {
				/* If channel is enabled for a direction */
				if (meas->conf.ch_list[ch].dirs & dirs) {
					kp_table_col(table, "");
				}
				continue;
			}
			kp_table_col(table, "0%*zu",
				     width, max_step_passes[ch][ne_dirs]);
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
				if (!(meas->conf.ch_list[ch].dirs & dirs &
				      kp_cap_dirs_from_ne(ne_dirs))) {
					/* If channel is enabled */
					if (meas->conf.ch_list[ch].dirs &
					    dirs) {
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
				for (char_idx = 0; char_idx <= width;
				     char_idx++) {
					if (char_idx == 0) {
						c = '|';
					} else if (char_idx == chars) {
						c = '|';
					} else if (char_idx == width) {
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

#undef STEP_NUM
}

/**
 * Output a measurement result to a shell.
 *
 * @param shell		The shell to output to.
 * @param meas		The measurement result to output.
 * @param verbose	True if the output should be verbose,
 * 			false otherwise.
 */
void
kp_meas_print(const struct shell *shell, const struct kp_meas *meas,
	      bool verbose)
{
	struct kp_table table;

	assert(shell != NULL);
	assert(kp_meas_is_valid(meas));

	/* Initialize the table output */
	kp_table_init(&table, shell,
		      KP_CAP_TIME_MAX_DIGITS + 1,
		      KP_CAP_CH_NAME_MAX_LEN,
		      1 + kp_meas_get_requested_ch_num(meas));

	/* Don't output anything, if got no data (no captured passes) */
	if (meas->captured_passes == 0) {
		return;
	}
	/* Output the index/name header */
	kp_meas_print_head(&table, meas);

	/* Output raw data, if verbose, or got only one captured pass */
	if (verbose || meas->captured_passes == 1) {
		kp_meas_print_data(&table, meas);
	}

	/* If got more than one captured pass */
	if (meas->captured_passes > 1) {
		/* Output stats */
		kp_meas_print_stats(&table, meas, verbose);
	}

	/* Output histogram */
	kp_meas_print_histogram(&table, meas, verbose);

	/* Add final separator */
	kp_table_sep(&table);
}

/**
 * Register a pass for making a measurement.
 *
 * @param meas	The measurement so far.
 * @param data	Opaque data.
 */
static void
kp_meas_make_pass(const struct kp_meas *meas, void *data)
{
	struct kp_table *table = (struct kp_table *)data;
	size_t pass = meas->passes - 1;

	assert(meas->passes > 0);

	/* If this is the first captured pass */
	/* NOTE: We promise we won't change it */
	if (kp_meas_get_ch_res((struct kp_meas *)meas, pass, 0) ==
		meas->ch_res_list &&
	    kp_meas_get_pass_ch_num(meas, pass) != 0) {
		/* Output the channel index/name header */
		kp_meas_print_head(table, meas);
		/* Output the raw data header */
		kp_meas_print_data_head(table, meas);
	}

	/* Output the pass, if it contains any data */
	kp_meas_print_data_pass(table, meas, pass);
}

enum kp_sample_rc
kp_meas_make(const struct shell *shell, struct kp_meas *meas, bool verbose)
{
	struct kp_table table;
	enum kp_sample_rc rc;

	assert(shell != NULL);
	assert(kp_meas_is_valid(meas));
	assert(kp_meas_is_empty(meas));

	/* Initialize the output table */
	kp_table_init(&table, shell,
		      KP_CAP_TIME_MAX_DIGITS + 1,
		      KP_CAP_CH_NAME_MAX_LEN,
		      1 + kp_meas_get_requested_ch_num(meas));

	/* Acquire the measurement, printing raw data if verbose */
	rc = kp_meas_acquire(meas,
			     (verbose ? kp_meas_make_pass : NULL),
			     &table);
	if (rc != KP_SAMPLE_RC_OK) {
		return rc;
	}

	/* Don't output anything, if got no data (no captured passes) */
	if (meas->captured_passes == 0) {
		return KP_SAMPLE_RC_OK;
	}

	/*
	 * Initialize the table, and print the channel header, if not
	 * done when printing raw pass data already.
	 */
	/* If haven't printed the headers in the pass function */
	if (!verbose) {
		/* Output the channel index/name header */
		kp_meas_print_head(&table, meas);
	}

	/* If got more than one captured pass */
	if (meas->captured_passes > 1) {
		/* Output stats */
		kp_meas_print_stats(&table, meas, verbose);
	/* Output raw data with header, if hadn't before */
	} else if (!verbose) {
		kp_meas_print_data(&table, meas);
	}

	/* Output histogram */
	kp_meas_print_histogram(&table, meas, verbose);

	/* Add final separator */
	kp_table_sep(&table);

	return KP_SAMPLE_RC_OK;
}
