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
#include "sw/device/silicon_creator/lib/error.h"

#include "flash_ctrl_regs.h"
#include "hw/ip/otp_ctrl/data/otp_ctrl_regs.h"

enum {
  /*
   * Maximum flash address, exclusive.
   */
  kMaxAddress =
      FLASH_CTRL_PARAM_BYTES_PER_BANK * FLASH_CTRL_PARAM_REG_NUM_BANKS,
  /*
   * The total number of flash pages.
   */
  kNumPages = kMaxAddress / FLASH_CTRL_PARAM_BYTES_PER_PAGE,

  // Let's say each page is 4 bytes and the ROM_EXT size is 16 bytes. By this
  // math, (16 / 4), the ROM_EXT is 4 pages. The size nicely happens to name the
  // index of the page immediately following the ROM_EXT.
  kPageAfterRomExtSlotA =
      CHIP_ROM_EXT_SIZE_MAX / FLASH_CTRL_PARAM_BYTES_PER_PAGE,
  kPageAfterRomExtSlotB = kPageAfterRomExtSlotA + kNumPages / 2,
};

// TODO(dmcardle) Delete this dead code
//
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

rom_error_t bootstrap_chip_erase(void) {
  // Bounds checks are defined at the granularity of pages, so we check
  // whether individual pages are safe to erase rather than erasing entire
  // banks at a time.
  flash_ctrl_bank_erase_perms_set(kHardenedBoolTrue);
  rom_error_t last_err = kErrorOk;

  for (uint32_t i = 0; i < kNumPages; ++i) {
    const uint32_t addr = i * FLASH_CTRL_PARAM_BYTES_PER_PAGE;
    // Do not erase this page if it lies within ROM_EXT on either slot.
    if (addr < CHIP_ROM_EXT_SIZE_MAX ||
        (addr >= FLASH_CTRL_PARAM_BYTES_PER_BANK &&
         addr < FLASH_CTRL_PARAM_BYTES_PER_BANK + CHIP_ROM_EXT_SIZE_MAX)) {
      continue;
    }
    rom_error_t err = flash_ctrl_data_erase(addr, kFlashCtrlEraseTypePage);
    if (err != kErrorOk) {
      last_err = err;
    }
  }
  flash_ctrl_bank_erase_perms_set(kHardenedBoolFalse);
  HARDENED_RETURN_IF_ERROR(last_err);
  return kErrorOk;
}

rom_error_t bootstrap_erase_verify(void) {
  rom_error_t last_err = kErrorOk;
  for (uint32_t i = 0; i < kNumPages; ++i) {
    const uint32_t addr = i * FLASH_CTRL_PARAM_BYTES_PER_PAGE;
    // Do not verify this page if it lies within ROM_EXT on either slot.
    if (addr < CHIP_ROM_EXT_SIZE_MAX ||
        (addr >= FLASH_CTRL_PARAM_BYTES_PER_BANK &&
         addr < FLASH_CTRL_PARAM_BYTES_PER_BANK + CHIP_ROM_EXT_SIZE_MAX)) {
      continue;
    }
    rom_error_t err =
        flash_ctrl_data_erase_verify(addr, kFlashCtrlEraseTypePage);
    if (err != kErrorOk) {
      last_err = err;
    }
  }
  HARDENED_RETURN_IF_ERROR(last_err);
  return last_err;
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

  enum {
    kNumPagesInRomExt = CHIP_ROM_EXT_SIZE_MAX / FLASH_CTRL_PARAM_BYTES_PER_PAGE
  };

  static_assert(kNumPagesInRomExt < FLASH_CTRL_PARAM_REG_PAGES_PER_BANK);

  // Disable erasing and programming of ROM_EXT in slot A.
  flash_ctrl_data_region_protect(/*region=*/0, /*page_offset=*/0,
                                 /*num_pages=*/kNumPagesInRomExt,
                                 /*erase_enabled=*/kMultiBitBool4False,
                                 /*prog_enabled=*/kMultiBitBool4False);

  // Disable erasing and programming of ROM_EXT in slot B.
  flash_ctrl_data_region_protect(
      /*region=*/1, /*page_offset=*/FLASH_CTRL_PARAM_REG_PAGES_PER_BANK,
      /*num_pages=*/kNumPagesInRomExt,
      /*erase_enabled=*/kMultiBitBool4False,
      /*prog_enabled=*/kMultiBitBool4False);

  // Disable erasing of the second flash bank.
  flash_ctrl_data_region_protect(
      /*region=*/2,
      /*page_offset=*/FLASH_CTRL_PARAM_REG_PAGES_PER_BANK + kNumPagesInRomExt,
      /*num_pages=*/FLASH_CTRL_PARAM_REG_PAGES_PER_BANK - kNumPagesInRomExt,
      /*erase_enabled=*/kMultiBitBool4False,
      /*prog_enabled=*/kMultiBitBool4False);

  return enter_bootstrap();
}
