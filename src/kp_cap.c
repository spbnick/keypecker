/** @file
 *  @brief Keypecker capturer
 */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "kp_cap.h"
#include "kp_misc.h"
#include <string.h>

/** Masks for the channels available for capture */
static const uint32_t kp_cap_ch_mask_list[KP_CAP_CH_NUM] = {
	LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3,
};

/** Capture interrupt-enabling/interrupt-flag masks for each channel */
/* NOTE: Assuming SR bits match DIER bits */
static const uint32_t kp_cap_ch_ccif_mask_list[KP_CAP_CH_NUM] = {
	TIM_SR_CC2IF,
	TIM_SR_CC3IF,
};

/** Overcapture flag masks for each channel */
static const uint32_t kp_cap_ch_ccof_mask_list[KP_CAP_CH_NUM] = {
	TIM_SR_CC2OF,
	TIM_SR_CC3OF,
};

/** Offset of the captured-value register for each channel, bytes */
static const size_t kp_cap_ch_ccr_offset_list[KP_CAP_CH_NUM] = {
	offsetof(TIM_TypeDef, CCR2),
	offsetof(TIM_TypeDef, CCR3),
};

/** The timer to use for capture */
static TIM_TypeDef* kp_cap_timer = NULL;

/** The semaphore signaling a capture can be started */
static K_SEM_DEFINE(kp_cap_available, 1, 1);

/** The capture interrupt mask for all channels to be captured */
static uint32_t kp_cap_ch_ccif_mask;

/** The maximum number of ticks to await capture of all channels */
static uint32_t kp_cap_timeout_ticks;

/** The minimum number of ticks to wait for a channel to bounce */
static uint32_t kp_cap_bounce_ticks;

/** The capture abort flag */
static volatile bool kp_cap_aborted;

/** The semaphore signaling a capture is done */
static K_SEM_DEFINE(kp_cap_done, 0, 1);

/**
 * The GPIO port to use for capture interrupt debugging for each channel.
 * Set to NULL if none. Must be configured if specified.
 */
static const struct device *kp_cap_ch_dbg_gpio_list[KP_CAP_CH_NUM];

/**
 * The GPIO pin to use for capture interrupt debugging for each channel.
 * Set high on entrance to the corresponding interrupt, set low on exit.
 * Only valid if the corresponding out_gpio_list element is not NULL.
 * Must be configured.
 */
static gpio_pin_t kp_cap_ch_dbg_pin_list[KP_CAP_CH_NUM];

/** The interrupt state spinlock */
/** Only needs to be held if kp_cap_available is taken */
static struct k_spinlock kp_cap_lock = {};

void
kp_cap_isr(void *arg)
{
	bool done = false;
	k_spinlock_key_t key;

	assert(kp_cap_is_initialized());

	key = k_spin_lock(&kp_cap_lock);

	/* If the capture is not aborted */
	if (!kp_cap_aborted) {
		uint32_t sr = kp_cap_timer->SR;
		/* If both the capture and bounce times have expired */
		if (sr & TIM_SR_UIF) {
			/* Disable the trigger */
			LL_TIM_SetSlaveMode(kp_cap_timer,
						LL_TIM_SLAVEMODE_DISABLED);
			/* Stop the timer */
			LL_TIM_DisableCounter(kp_cap_timer);
			/* Disable all the interrupts */
			kp_cap_timer->DIER = 0;
			/* Prepare to signal the capture is done */
			done = true;
		} else {
			uint32_t ccif_mask = sr & kp_cap_ch_ccif_mask;
			uint32_t new_ccif_mask = ccif_mask & kp_cap_timer->DIER;
			size_t i;

			/* Lower debugging GPIO pins, if specified */
			for (i = 0; i < KP_CAP_CH_NUM; i++) {
				if (kp_cap_ch_dbg_gpio_list[i] != NULL &&
				    (kp_cap_ch_ccif_mask_list[i] &
					new_ccif_mask)) {
					gpio_pin_set(
						kp_cap_ch_dbg_gpio_list[i],
						kp_cap_ch_dbg_pin_list[i],
						0
					);
				}
			}

			/* If all channels were captured (but may bounce) */
			if (ccif_mask == kp_cap_ch_ccif_mask) {
				/* Shorten the capture, if possible */
				LL_TIM_DisableCounter(kp_cap_timer);
				if ((uint32_t)kp_cap_timer->CNT <
						kp_cap_timeout_ticks) {
					LL_TIM_SetAutoReload(
						kp_cap_timer,
						kp_cap_timer->CNT +
						kp_cap_bounce_ticks
					);
				}
				LL_TIM_EnableCounter(kp_cap_timer);
			}

			/* Disable the interrupts we've processed */
			kp_cap_timer->DIER &= ~ccif_mask;
		}
	}

	k_spin_unlock(&kp_cap_lock, key);

	if (done) {
		/* Signal the capture is done */
		k_sem_give(&kp_cap_done);
	}
}

void
kp_cap_start(const struct kp_cap_ch_conf *ch_conf_list,
		size_t ch_conf_num, enum kp_cap_dir dir,
		uint32_t timeout_us, uint32_t bounce_us)
{
	size_t i;
	const struct kp_cap_ch_conf *ch_conf;
	uint32_t ch_mask;
	k_spinlock_key_t key;

	assert(kp_cap_is_initialized());
	assert(ch_conf_list != NULL || ch_conf_num == 0);
	assert(kp_cap_dir_is_valid(dir));
	assert(timeout_us + bounce_us <= KP_CAP_TIME_MAX_US);

	/* Wait for the capture to be available */
	k_sem_take(&kp_cap_available, K_FOREVER);

	/* Lock the interrupt state */
	key = k_spin_lock(&kp_cap_lock);

	/* Initialize the capture configuration */
	kp_cap_ch_ccif_mask = 0;

	/* For each channel */
	for (i = 0; i < KP_CAP_CH_NUM; i++) {
		ch_mask = kp_cap_ch_mask_list[i];
		/* NOTE: Must be considered invalid before the check below */
		ch_conf = &ch_conf_list[i];
		/* If the channel's capture is enabled */
		if (i < ch_conf_num && (ch_conf->dir & dir)) {
			/* Configure and enable the capture */
			kp_cap_ch_ccif_mask |= kp_cap_ch_ccif_mask_list[i];
			LL_TIM_IC_Config(
				kp_cap_timer, ch_mask,
				LL_TIM_ACTIVEINPUT_DIRECTTI |
				LL_TIM_ICPSC_DIV1 |
				LL_TIM_IC_FILTER_FDIV1 |
				(ch_conf->rising
					? LL_TIM_IC_POLARITY_RISING
					: LL_TIM_IC_POLARITY_FALLING)
			);
			LL_TIM_CC_EnableChannel(kp_cap_timer, ch_mask);
			/* Record debug GPIO device and pin, if any */
			kp_cap_ch_dbg_gpio_list[i] = ch_conf->dbg_gpio;
			kp_cap_ch_dbg_pin_list[i] = ch_conf->dbg_pin;
			/* Raise the debugging pin, if any */
			if (kp_cap_ch_dbg_gpio_list[i] != NULL) {
				gpio_pin_set(kp_cap_ch_dbg_gpio_list[i],
						kp_cap_ch_dbg_pin_list[i], 1);
			}
		} else {
			/* Disable the capture */
			LL_TIM_CC_DisableChannel(kp_cap_timer, ch_mask);
			/* Clear debug GPIO device */
			kp_cap_ch_dbg_gpio_list[i] = NULL;
		}
	}

	/* Reset abort flag */
	kp_cap_aborted = false;

	/* Remember the number of ticks to wait for capture */
	kp_cap_timeout_ticks = timeout_us / KP_CAP_RES_US;

	/* Remember the number of ticks to wait for a channel to bounce */
	kp_cap_bounce_ticks = bounce_us / KP_CAP_RES_US;

	/* Clear all the overcapture/interrupt flags */
	kp_cap_timer->SR = 0;

	/* Enable only the interrupts we need */
	/* NOTE: Assuming SR bits match DIER bits */
	kp_cap_timer->DIER = kp_cap_ch_ccif_mask | TIM_SR_UIF;

	/* Set auto-reload register to the total timeout */
	LL_TIM_SetAutoReload(kp_cap_timer,
			     kp_cap_timeout_ticks +
			     kp_cap_bounce_ticks);

	/* Setup the trigger to start (but not stop) counting */
	LL_TIM_SetSlaveMode(kp_cap_timer, LL_TIM_SLAVEMODE_TRIGGER);

	/* Unlock the interrupt state */
	k_spin_unlock(&kp_cap_lock, key);
}

bool
kp_cap_abort(void)
{
	bool aborted;
	k_spinlock_key_t key;

	assert(kp_cap_is_initialized());

	/* Lock the interrupt state */
	key = k_spin_lock(&kp_cap_lock);

	/* If the timer is primed for triggering, i.e. we're capturing */
	if ((kp_cap_timer->SMCR & TIM_SMCR_SMS) == LL_TIM_SLAVEMODE_TRIGGER) {
		/* Disable the trigger */
		LL_TIM_SetSlaveMode(kp_cap_timer, LL_TIM_SLAVEMODE_DISABLED);
		/* Stop the timer */
		LL_TIM_DisableCounter(kp_cap_timer);
		/* Disable all the interrupts */
		kp_cap_timer->DIER = 0;
		/* Mark capture as aborted */
		kp_cap_aborted = true;
		/* Report as aborted */
		aborted = true;
	} else {
		/* Report as not aborted */
		aborted = false;
	}

	/* Unlock the interrupt state */
	k_spin_unlock(&kp_cap_lock, key);

	if (aborted) {
		/* Signal the capture is done */
		k_sem_give(&kp_cap_done);
	}

	return aborted;
}

void
kp_cap_finish_event_init(struct k_poll_event *event)
{
	assert(kp_cap_is_initialized());
	k_poll_event_init(event, K_POLL_TYPE_SEM_AVAILABLE,
			  K_POLL_MODE_NOTIFY_ONLY, &kp_cap_done);
}

enum kp_cap_rc
kp_cap_finish(struct kp_cap_ch_res *ch_res_list,
		size_t ch_res_num, k_timeout_t timeout)
{
	size_t i;
	enum kp_cap_ch_status status;
	uint32_t value_ticks;
	uint32_t value_us;

	assert(kp_cap_is_initialized());
	assert(ch_res_list != NULL || ch_res_num == 0);

	/* Wait for the capture to be done */
	if (k_sem_take(&kp_cap_done, timeout)) {
		return KP_CAP_RC_TIMEOUT;
	}

	/* If the capture was aborted */
	if (kp_cap_aborted) {
		/* Allow another capture */
		k_sem_give(&kp_cap_available);
		return KP_CAP_RC_ABORTED;
	}

	/* Initialize results to all-disabled */
	memset(ch_res_list, 0, sizeof(*ch_res_list) * ch_res_num);

	/* For each channel */
	for (i = 0; i < KP_CAP_CH_NUM; i++) {
		/* Skip disabled channels */
		if (!LL_TIM_CC_IsEnabledChannel(kp_cap_timer,
						kp_cap_ch_mask_list[i])) {
			continue;
		}

		/* If the channel was captured */
		if (kp_cap_timer->SR & kp_cap_ch_ccif_mask_list[i]) {
			/* Read the value (clears the capture flag) */
			value_ticks = *(uint32_t *)(
				(uint8_t *)kp_cap_timer +
				kp_cap_ch_ccr_offset_list[i]
			);
			value_us = value_ticks * KP_CAP_RES_US;
			/* If the channel was over-captured */
			if (kp_cap_timer->SR & kp_cap_ch_ccof_mask_list[i]) {
				status = KP_CAP_CH_STATUS_OVERCAPTURE;
				/* Reset the overcapture flag */
				kp_cap_timer->SR &=
					~kp_cap_ch_ccof_mask_list[i];
			/* Else, if the channel was captured late */
			} else if (value_ticks > kp_cap_timeout_ticks) {
				status = KP_CAP_CH_STATUS_TIMEOUT;
			} else {
				status = KP_CAP_CH_STATUS_OK;
			}
		/* Else, channel capture has timed out */
		} else {
			status = KP_CAP_CH_STATUS_TIMEOUT;
			value_us = UINT32_MAX;
		}

		/* If the channel's result is requested */
		if (i < ch_res_num) {
			ch_res_list[i].status = status;
			ch_res_list[i].value_us = value_us;
		}
	}

	/* Allow another capture */
	k_sem_give(&kp_cap_available);

	return KP_CAP_RC_OK;
}

bool
kp_cap_dir_from_str(const char *str, enum kp_cap_dir *pdir)
{
	enum kp_cap_dir dir;
	assert(str != NULL);

	if (kp_strcasecmp(str, "none") == 0) {
		dir = KP_CAP_DIR_NONE;
	} else if (kp_strcasecmp(str, "up") == 0) {
		dir = KP_CAP_DIR_UP;
	} else if (kp_strcasecmp(str, "down") == 0) {
		dir = KP_CAP_DIR_DOWN;
	} else if (kp_strcasecmp(str, "both") == 0) {
		dir = KP_CAP_DIR_BOTH;
	} else {
		return false;
	}

	if (pdir != NULL) {
		*pdir = dir;
	}

	return true;
}

const char *
kp_cap_dir_to_lcstr(enum kp_cap_dir dir)
{
	static const char *str_list[] = {
#define STATUS(_token, _lc_token) [KP_CAP_DIR_##_token] = #_lc_token
		STATUS(NONE, none),
		STATUS(UP, up),
		STATUS(DOWN, down),
		STATUS(BOTH, both),
#undef STATUS
	};
	const char *str = (dir >= 0 && dir < ARRAY_SIZE(str_list))
		? str_list[dir] : NULL;
	return str == NULL ? "unknown" : str;
}

const char *
kp_cap_dir_to_cpstr(enum kp_cap_dir dir)
{
	static const char *str_list[] = {
#define STATUS(_token, _cp_token) [KP_CAP_DIR_##_token] = #_cp_token
		STATUS(NONE, None),
		STATUS(UP, Up),
		STATUS(DOWN, Down),
		STATUS(BOTH, Both),
#undef STATUS
	};
	const char *str = (dir >= 0 && dir < ARRAY_SIZE(str_list))
		? str_list[dir] : NULL;
	return str == NULL ? "unknown" : str;
}

const char *
kp_cap_ch_status_to_str(enum kp_cap_ch_status status)
{
	static const char *str_list[KP_CAP_CH_STATUS_NUM] = {
#define STATUS_STR(_token) [KP_CAP_CH_STATUS_##_token] = #_token
		STATUS_STR(DISABLED),
		STATUS_STR(TIMEOUT),
		STATUS_STR(OK),
		STATUS_STR(OVERCAPTURE),
#undef STATUS_STR
	};
	const char *str = (status >= 0 && status < ARRAY_SIZE(str_list))
		? str_list[status] : NULL;
	return str == NULL ? "UNKNOWN" : str;
}

const char *
kp_cap_rc_to_str(enum kp_cap_rc rc)
{
	static const char *str_list[KP_CAP_RC_NUM] = {
#define RC_STR(_token) [KP_CAP_RC_##_token] = #_token
		RC_STR(OK),
		RC_STR(ABORTED),
		RC_STR(TIMEOUT),
#undef RC_STR
	};
	const char *str = (rc >= 0 && rc < ARRAY_SIZE(str_list))
		? str_list[rc] : NULL;
	return str == NULL ? "UNKNOWN" : str;
}

bool
kp_cap_is_initialized(void)
{
	return kp_cap_timer != NULL;
}

void
kp_cap_init(TIM_TypeDef* timer)
{
	assert(!kp_cap_is_initialized());
	assert(timer != NULL);

	/* Remember the timer we're using */
	kp_cap_timer = timer;

	/* Set update interrupt generation for overflow/underflow only */
	LL_TIM_SetUpdateSource(kp_cap_timer, LL_TIM_UPDATESOURCE_COUNTER);
	/* Setup prescaling to get our resolution with the system clock */
	LL_TIM_SetPrescaler(kp_cap_timer, k_us_to_cyc_floor32(KP_CAP_RES_US));
	/* Update prescaler */
	LL_TIM_GenerateEvent_UPDATE(kp_cap_timer);
	/* Count up from zero */
	LL_TIM_SetCounterMode(kp_cap_timer, LL_TIM_COUNTERMODE_UP);
	/* Configure trigger channel as input:
	 * directly connected to its input line,
	 * no prescaling (not used for triggering anyway),
	 * no filtering,
	 * rising edge detection
	 */
	LL_TIM_IC_Config(kp_cap_timer, LL_TIM_CHANNEL_CH1,
			 LL_TIM_ACTIVEINPUT_DIRECTTI |
			 LL_TIM_ICPSC_DIV1 |
			 LL_TIM_IC_FILTER_FDIV1 |
			 LL_TIM_IC_POLARITY_RISING);
	/* Set filtered and polarity-configured channel as trigger */
	LL_TIM_SetTriggerInput(kp_cap_timer, LL_TIM_TS_TI1FP1);
	/* Setup trigger to start (but not stop) counting */
	LL_TIM_SetSlaveMode(kp_cap_timer, LL_TIM_SLAVEMODE_TRIGGER);

	assert(kp_cap_is_initialized());
}
