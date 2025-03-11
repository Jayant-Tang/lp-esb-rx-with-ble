#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

# Add configs
list(APPEND EXTRA_CONF_FILE
  "${CMAKE_CURRENT_LIST_DIR}/src/mpsl_esb/mpsl-esb-overlay.conf"
)

set(EXTRA_CONF_FILE ${EXTRA_CONF_FILE} CACHE INTERNAL "")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_esb_low_power)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
)

add_subdirectory(src/mpsl_esb)

# NORDIC SDK APP END
