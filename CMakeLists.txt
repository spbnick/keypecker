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
	src/kp_input.c
	src/kp_act.c
	src/kp_cap.c
	src/kp_sample.c
	src/kp_meas.c
	src/kp_table.c
)
