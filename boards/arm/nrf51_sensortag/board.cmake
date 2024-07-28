# SPDX-License-Identifier: Apache-2.0
set (OPENOCD_NRF5_SUBFAMILY "nrf51")
board_runner_args(pyocd "--target=nrf51")
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-nrf5.board.cmake)
