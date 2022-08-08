# SPDX-License-Identifier: Apache-2.0
board_runner_args(pyocd "--target=stm32g030f6p6")
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset=hw")
board_runner_args(jlink "--device=STM32G030F6" "--speed=4000")
board_runner_args(openocd --cmd-pre-init "source [find interface/jlink.cfg]")
board_runner_args(openocd --cmd-pre-init "source [find target/stm32g0x.cfg]")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
