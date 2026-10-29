# Copyright (c) 2026 Mecha Systems
# SPDX-License-Identifier: BSD-3-Clause

zephyr_library()
zephyr_library_property(ALLOW_EMPTY TRUE)
zephyr_library_include_directories(${ZEPHYR_MECHA_PIVOT_PIP_MODULE_DIR}/lib/pip_core)

zephyr_library_sources_ifdef(CONFIG_INPUT_MECHA_PIVOT_PIP input_mecha_pivot_pip.c)
