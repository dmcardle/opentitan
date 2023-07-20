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

extern const hardened_bool_t kProtectRomExt;

/**
 * Handles access permissions and erases both data banks of the embedded flash.
 *
 * NOTE: Code that depends on this library must provide an implementation of
 * this function.
 *
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
rom_error_t bootstrap_chip_erase(void);

/**
 * Handles access permissions and erases a 4 KiB region in the data partition of
 * the embedded flash.
 *
 * Since OpenTitan's flash page size is 2 KiB, this function erases two
 * consecutive pages.
 *
 * NOTE: Code that depends on this library must provide an implementation of
 * this function.
 *
 * @param addr Address that falls within the 4 KiB region being deleted.
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
rom_error_t bootstrap_sector_erase(uint32_t addr);

/**
 * Verify that all data banks have been erased.
 *
 * This function also clears the WIP and WEN bits of the flash status register.
 *
 * NOTE: Code that depends on this library must provide an implementation of
 * this function.
 *
 * @param[in,out] ctx State machine context.
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
rom_error_t bootstrap_erase_verify(void);

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
