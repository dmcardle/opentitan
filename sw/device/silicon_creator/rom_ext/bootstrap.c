// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sw/device/silicon_creator/rom_ext/bootstrap.h"

#include <stdint.h>

#include "sw/device/lib/base/hardened.h"
#include "sw/device/silicon_creator/lib/base/chip.h"
#include "sw/device/silicon_creator/lib/bootstrap.h"
#include "sw/device/silicon_creator/lib/drivers/flash_ctrl.h"
#include "sw/device/silicon_creator/lib/drivers/otp.h"
#include "sw/device/silicon_creator/lib/drivers/rstmgr.h"
#include "sw/device/silicon_creator/lib/drivers/spi_device.h"
#include "sw/device/silicon_creator/lib/error.h"

#include "flash_ctrl_regs.h"
#include "hw/ip/otp_ctrl/data/otp_ctrl_regs.h"

// This function implements the prototype declared by
// sw/device/silicon_creator/lib/bootstrap.h
hardened_bool_t bootstrap_bounds_check(spi_device_opcode_t opcode,
                                       uint32_t page_addr) {
  static_assert(CHIP_ROM_EXT_SIZE_MAX % FLASH_CTRL_PARAM_BYTES_PER_PAGE == 0,
                "CHIP_ROM_EXT_SIZE_MAX must be aligned to flash page boundary");

  switch (opcode) {
    case kSpiDeviceOpcodeChipErase:
    case kSpiDeviceOpcodeSectorErase:
      // Check whether `page_addr` lies within slot A, but outside of ROM_EXT.
      if (page_addr >= CHIP_ROM_EXT_SIZE_MAX &&
          page_addr < FLASH_CTRL_PARAM_BYTES_PER_BANK) {
        return kHardenedBoolTrue;
      }
      // Check whether `page_addr` lies within slot B, but outside of ROM_EXT.
      if (page_addr >=
              FLASH_CTRL_PARAM_BYTES_PER_BANK + CHIP_ROM_EXT_SIZE_MAX &&
          page_addr < 2 * FLASH_CTRL_PARAM_BYTES_PER_BANK) {
        return kHardenedBoolTrue;
      }
      break;
    case kSpiDeviceOpcodePageProgram:
      // Check whether `page_addr` lies within slot A, but outside of ROM_EXT.
      if (page_addr >= CHIP_ROM_EXT_SIZE_MAX &&
          page_addr < FLASH_CTRL_PARAM_BYTES_PER_BANK) {
        return kHardenedBoolTrue;
      }
      break;
    default:
      return kHardenedBoolFalse;
  }
  return kHardenedBoolFalse;
}

hardened_bool_t rom_ext_bootstrap_enabled(void) {
  // Check that bootstrap is enabled in OTP.
  uint32_t bootstrap_en =
      otp_read32(OTP_CTRL_PARAM_OWNER_SW_CFG_ROM_EXT_BOOTSTRAP_EN_OFFSET);
  if (bootstrap_en != kHardenedBoolTrue) {
    return kHardenedBoolFalse;
  }
  HARDENED_CHECK_EQ(bootstrap_en, kHardenedBoolTrue);

  // Check that the reset reason is PoR.
  const uint32_t reset_mask = 1 << kRstmgrReasonPowerOn;
  const uint32_t reset_reason = rstmgr_reason_get();
  if ((reset_reason & reset_mask) == 0) {
    return kHardenedBoolFalse;
  }
  HARDENED_CHECK_NE(reset_reason & reset_mask, 0);
  return kHardenedBoolTrue;
}

rom_error_t rom_ext_bootstrap(void) {
  hardened_bool_t enabled = rom_ext_bootstrap_enabled();
  if (launder32(enabled) != kHardenedBoolTrue) {
    return kErrorBootstrapDisabledRomExt;
  }
  HARDENED_CHECK_EQ(enabled, kHardenedBoolTrue);

  return enter_bootstrap(/*enable_bounds_checks=*/kHardenedBoolTrue);
}
