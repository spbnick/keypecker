/* kp.c - Manic Typer main entry point */

/*
 * Copyright (c) 2022 Nikolai Kondrashov
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <zephyr.h>
#include <device.h>
#include <assert.h>
#include <logging/log.h>

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(kp);

void
main(void)
{
    printk("It's alive!\n");
}
