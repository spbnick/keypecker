/** @file
 *  @brief Keypecker measurement
 */

/*
 * Copyright (c) 2023 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_MEASURE_H_
#define KP_MEASURE_H_

#include "kp_sample.h"
#include "kp_cap.h"
#include <zephyr/shell/shell.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A measurement in progress */
struct kp_meas {
	/* Capture configuration */
	struct kp_cap_conf conf;
	/* Top position of the movement range (< bottom) */
	int32_t	top;
	/* Bottom position of the movement range (> top) */
	int32_t	bottom;
	/* The speed with which to move, 0-100% */
	uint32_t speed;
	/* Number of passes that should be done */
	size_t requested_passes;
	/* True if even passes are going down, false if up */
	bool even_down;
	/* Number of passes with captured channel results so far */
	size_t captured_passes;
	/* Number of all passes done so far */
	size_t passes;
	/* List of channel capture results for passes so far */
	struct kp_cap_ch_res ch_res_list[1024];
};

/** An invalid measurement initializer (top == bottom) */
#define KP_MEAS_INVALID	(struct kp_meas){0,}

/**
 * Check if a measurement is valid.
 *
 * @param meas	The measurement to check.
 *
 * @return True if the measurement is valid, false otherwise.
 */
static inline bool
kp_meas_is_valid(const struct kp_meas *meas)
{
	return meas != NULL &&
	       kp_cap_conf_is_valid(&meas->conf) &&
	       kp_act_pos_is_valid(meas->top) &&
	       kp_act_pos_is_valid(meas->bottom) &&
	       meas->top < meas->bottom &&
	       meas->speed <= 100 &&
	       (meas->even_down & 1) == meas->even_down &&
	       meas->passes <= meas->requested_passes &&
	       kp_cap_conf_ch_num(&meas->conf, KP_CAP_DIRS_BOTH) > 0 &&
	       kp_cap_conf_ch_res_idx(&meas->conf, meas->even_down,
				      meas->passes, 0) <=
		       ARRAY_SIZE(meas->ch_res_list);
}

/**
 * Check if a measurement is null, i.e. it won't capture any results.
 *
 * @param meas	The measurement to check. Must be valid.
 *
 * @return True if the measurement is null, false otherwise.
 */
static inline bool
kp_meas_is_null(const struct kp_meas *meas)
{
	assert(kp_meas_is_valid(meas));
	return kp_cap_conf_ch_res_idx(&meas->conf, meas->even_down,
				      meas->requested_passes, 0) == 0;
}

/**
 * Check if a measurement is empty.
 *
 * @param meas	The measurement to check. Must be valid.
 *
 * @return True if the measurement is empty, false otherwise.
 */
static inline bool
kp_meas_is_empty(const struct kp_meas *meas)
{
	assert(kp_meas_is_valid(meas));
	return meas->passes == 0;
}

/**
 * Initialize an empty measurement for a number of passes over a range of
 * actuator positions. The caller must ensure all requested capture results
 * can be accommodated.
 *
 * @param meas		The measurement to initialize.
 * @param top		The top position of the movement range.
 * 			Must be less than the bottom.
 * @param bottom	The bottom position of the movement range.
 * 			Must be greater than the top.
 * @param speed		The speed with which to move, 0-100%.
 * @param passes	Number of actuator passes to execute.
 * @param conf		The capture configuration to use.
 * 			Must be valid, and have at least one channel enabled
 * 			in at least one direction.
 * @param even_down	True if even passes must be going down, false if up.
 */
static inline void
kp_meas_init(struct kp_meas *meas,
	     int32_t top, int32_t bottom,
	     uint32_t speed, size_t passes,
	     const struct kp_cap_conf *conf,
	     bool even_down)
{
	assert(meas != NULL);
	assert(kp_act_pos_is_valid(top));
	assert(kp_act_pos_is_valid(bottom));
	assert(top < bottom);
	assert(speed <= 100);
	assert(kp_cap_conf_is_valid(conf));
	assert((even_down & 1) == even_down);
	assert(kp_cap_conf_ch_num(conf, KP_CAP_DIRS_BOTH) > 0);
	assert(kp_cap_conf_ch_res_idx(conf, even_down, passes, 0) <=
	       ARRAY_SIZE(meas->ch_res_list));

	meas->conf = *conf;
	meas->top = top;
	meas->bottom = bottom;
	meas->speed = speed;
	meas->requested_passes = passes;
	meas->even_down = even_down;
	meas->captured_passes = 0;
	meas->passes = 0;

	assert(kp_meas_is_valid(meas));
	assert(kp_meas_is_empty(meas));
}

/**
 * Get the requested set of directions for a measurement.
 *
 * @param meas	The measurement to get directions for.
 *
 * @return The requested set of measurement directions.
 */
static inline enum kp_cap_dirs
kp_meas_get_requested_dirs(const struct kp_meas *meas)
{
	assert(kp_meas_is_valid(meas));
	return (meas->requested_passes == 0)
		? KP_CAP_DIRS_NONE
		: ((meas->requested_passes == 1)
			? kp_cap_dirs_from_down(meas->even_down)
			: KP_CAP_DIRS_BOTH);
}

/**
 * Get the number of all channels requested to be captured for a measurement.
 *
 * @param meas	The measurement to get channel number for.
 *
 * @return The number of channels requested for capturing.
 */
static inline size_t
kp_meas_get_requested_ch_num(const struct kp_meas *meas)
{
	assert(kp_meas_is_valid(meas));
	return kp_cap_conf_ch_num(&meas->conf,
				  kp_meas_get_requested_dirs(meas));
}

/**
 * Get a measurement pass (unit) direction set.
 *
 * @param meas	The measurement to get a pass direction set for.
 * @param pass	The pass to get the (unit) direction set for.
 *		Must be less than the number of passes in the measurement.
 *
 * @return The unit direction set of the measurement pass.
 */
static inline size_t
kp_meas_get_pass_dir(const struct kp_meas *meas, size_t pass)
{
	assert(kp_meas_is_valid(meas));
	assert(pass < meas->passes);
	return kp_cap_dirs_from_down((pass ^ meas->even_down) & 1);
}

/**
 * Get the number of captured channels for a specific measurement pass.
 *
 * @param meas	The measurement to get channel count for.
 * @param pass	The index of the pass to get the channel count for.
 *		Must be less than the number of passes in the measurement.
 *
 * @return The number of channels captured for the measurement pass.
 */
static inline size_t
kp_meas_get_pass_ch_num(const struct kp_meas *meas, size_t pass)
{
	assert(kp_meas_is_valid(meas));
	assert(pass < meas->passes);
	return kp_cap_conf_ch_num(
		&meas->conf,
		kp_meas_get_pass_dir(meas, pass)
	);
}

/**
 * Get the pointer to a channel result in the channel result list of the
 * specified measurement, for the specified pass and channel indices.
 *
 * @param meas	The measurement to get the channel result from.
 * @param pass	The index of the pass to get the channel result for.
 *		Must be less than the number of passes in the measurement.
 * @param ch	The index of the channel to get the result for.
 *		Must be less than the number of capture channels.
 *
 * @return The pointer to the result of the channel in the measurement's pass.
 * 	   NOTE: might be pointing to another channel's result, if this
 * 	   specific channel wasn't captured for the pass.
 */
static inline struct kp_cap_ch_res *
kp_meas_get_ch_res(struct kp_meas *meas, size_t pass, size_t ch)
{
	assert(kp_meas_is_valid(meas));
	assert(pass < meas->passes);
	assert(ch < ARRAY_SIZE(meas->conf.ch_list));
	return meas->ch_res_list +
		kp_cap_conf_ch_res_idx(&meas->conf, meas->even_down, pass, ch);
}

/**
 * Prototype for a function notifying about an acquired pass.
 *
 * @param meas	The measurement so far.
 * @param data	Opaque data.
 */
typedef void (*kp_meas_acquire_pass_fn)(const struct kp_meas *meas,
					void *data);

/**
 * Acquire an initalized measurement.
 *
 * @param meas		The measurement to acquire.
 *			Must be initialized and empty.
 * @param pass_fn	The function to call for every pass-worth of samples.
 * 			Can be NULL to have nothing called.
 * @param pass_data	The data to pass to pass_fn with each call.
 *
 * @return Sampling result code.
 */
extern enum kp_sample_rc kp_meas_acquire(struct kp_meas *meas,
					 kp_meas_acquire_pass_fn pass_fn,
					 void *pass_data);

/**
 * Output a measurement result to a shell.
 *
 * @param shell		The shell to output to.
 * @param meas		The measurement result to output.
 * @param verbose	True if the output should be verbose,
 * 			false otherwise.
 */
extern void kp_meas_print(const struct shell *shell,
			  const struct kp_meas *meas,
			  bool verbose);

/**
 * Make (acquire and print) an initialized measurement.
 *
 * @param shell		The shell to output the measurement to.
 * @param meas		The measurement to acquire and print.
 *			Must be initialized and empty.
 * @param verbose	True if the output should be verbose,
 * 			false otherwise.
 */
extern enum kp_sample_rc kp_meas_make(const struct shell *shell,
				      struct kp_meas *meas,
				      bool verbose);

#ifdef __cplusplus
}
#endif

#endif /* KP_MEASURE_H_ */
