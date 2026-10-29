# Copyright (c) 2026 Mecha Systems
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pip_core)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE ${ZEPHYR_MECHA_PIVOT_PIP_MODULE_DIR}/lib/pip_core)
