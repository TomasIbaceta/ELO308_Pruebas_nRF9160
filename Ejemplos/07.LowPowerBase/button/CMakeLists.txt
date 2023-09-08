# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(button)

# a forum poster implies that putting this line and creating a spm.conf allows you to disable uart in the trusted zone too.
set(spm_CONF_FILE
    prj.conf
    $(CMAKE_CURRENT_LIST_DIR)/spm.conf
    )

# Rest of the program
target_sources(app PRIVATE src/main.c)

