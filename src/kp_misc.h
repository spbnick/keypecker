/** @file
 *  @brief Keypecker - miscellaneous definitions
 */

/*
 * Copyright (c) 2023 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef KP_MISC_H_
#define KP_MISC_H_

#include <zephyr/sys/util.h>
#include <strings.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline int
kp_strcasecmp(const char *a, const char *b)
{
	return strncasecmp(a, b, MAX(strlen(a), strlen(b)));
}

#ifdef __cplusplus
}
#endif

#endif /* KP_MISC_H_ */
