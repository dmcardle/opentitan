// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
#ifndef OPENTITAN_SW_DEVICE_SILICON_CREATOR_LIB_BOOTSTRAP_H_
#define OPENTITAN_SW_DEVICE_SILICON_CREATOR_LIB_BOOTSTRAP_H_

#include <stdint.h>

#include "sw/device/lib/base/hardened.h"
#include "sw/device/silicon_creator/lib/drivers/spi_device.h"
#include "sw/device/silicon_creator/lib/error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Determines whether the given SPI opcode is allowed to operate on the given
 * page.
 *
 * NOTE: Code that depends on this library must provide an implementation of
 * this function.
 *
 * @param opcode The current SPI command's opcode.
 * @param addr The flash address in question.
 * @return Whether the address passes the bounds check.
 */
OT_WARN_UNUSED_RESULT
hardened_bool_t bootstrap_bounds_check(spi_device_opcode_t opcode,
                                       uint32_t page_addr);

/**
 * Enters flash programming mode. This function initializes the SPI device and
 * uses incoming SPI commands to drive an internal state machine.
 *
 * Bootstrapping uses the typical SPI flash EEPROM commands. A typical session
 * involves:
 * - Asserting bootstrap pins to enter bootstrap mode,
 * - Erasing the chip (WREN, CHIP_ERASE, busy loop ...),
 * - Programming the chip (WREN, PAGE_PROGRAM, busy loop ...), and
 * - Resetting the chip (RESET).
 *
 * When bounds checks are enabled, this function will consult
 * `bootstrap_bounds_check()` to determine whether operations are allowed.
 *
 * This function only returns on error; a successful session ends with a chip
 * reset.
 *
 * @param enable_bounds_checks
 * @return The result of the flash loop.
 */
OT_WARN_UNUSED_RESULT
rom_error_t enter_bootstrap(hardened_bool_t enable_bounds_checks);

#ifdef __cplusplus
}
#endif

#endif  // OPENTITAN_SW_DEVICE_SILICON_CREATOR_LIB_BOOTSTRAP_H_
