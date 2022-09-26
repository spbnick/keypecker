# SPDX-License-Identifier: GPL-2.0-or-later
cmake_minimum_required(VERSION 3.20.1)
set(BOARD stm32_min_dev_blue)
set(DTC_OVERLAY_FILE dts.overlay)
find_package(Zephyr REQUIRED)
project(keypecker VERSION 1)
target_sources(
	app PRIVATE
	src/kp.c
	src/kp_shell.c
	src/kp_act.c
)
