/** @file
 *  @brief Keypecker sample handling
 */

/*
 * Copyright (c) 2023 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_SAMPLE_H_
#define KP_SAMPLE_H_

#include "kp_input.h"
#include "kp_act.h"
#include "kp_cap.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Sampling result code */
enum kp_sample_rc {
	/* Success */
	KP_SAMPLE_RC_OK,
	/* Aborted */
	KP_SAMPLE_RC_ABORTED,
	/* Actuator is off */
	KP_SAMPLE_RC_OFF,
};

/**
 * Sample captured channels for a specified movement.
 *
 * @param target	The absolute actuator position to move to.
 * @param speed		The speed with which to move, 0-100%.
 * @param conf		The capture configuration to use.
 * @param dirs		The capture movement directions.
 * @param ch_res_list	Location for channel capture results.
 * 			Only results for channels enabled in the
 * 			capture configuration for the specified directions (as
 * 			counted by kp_cap_conf_ch_num()) will be output.
 * 			Can be NULL, if ch_res_num is zero.
 * @param ch_res_num	Maximum number of channel results to output into
 *			"ch_res_list".
 *
 * @return Result code.
 */
extern enum kp_sample_rc kp_sample(int32_t target,
				   uint32_t speed,
				   const struct kp_cap_conf *conf,
				   enum kp_cap_dirs dirs,
				   struct kp_cap_ch_res *ch_res_list,
				   size_t ch_res_num);

/**
 * Count the number of all-enabled-channel triggers for a number of passes
 * over a range of actuator positions.
 *
 * @param top		The top position of the range.
 * @param bottom	The bottom position of the range.
 * @param speed		The speed with which to move, 0-100%.
 * @param passes	Number of actuator passes to execute.
 * @param conf		The capture configuration to use.
 * 			Must be valid, and have at least one channel enabled
 * 			in at least one direction.
 * @param ptriggers	Location for the number of triggered passes.
 * 			Can be NULL to have the number discarded.
 *
 * @return Sampling result code.
 */
extern enum kp_sample_rc kp_sample_check(int32_t top, int32_t bottom,
					 uint32_t speed, size_t passes,
					 const struct kp_cap_conf *conf,
					 size_t *ptriggers);

#ifdef __cplusplus
}
#endif

#endif /* KP_SAMPLE_H_ */
