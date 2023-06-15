// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sw/device/silicon_creator/lib/bootstrap.h"

#include <assert.h>
#include <stdalign.h>
#include <stdint.h>

#include "sw/device/lib/base/bitfield.h"
#include "sw/device/lib/base/hardened.h"
#include "sw/device/silicon_creator/lib/base/chip.h"
#include "sw/device/silicon_creator/lib/drivers/flash_ctrl.h"
#include "sw/device/silicon_creator/lib/drivers/rstmgr.h"
#include "sw/device/silicon_creator/lib/drivers/spi_device.h"
#include "sw/device/silicon_creator/lib/error.h"

#include "flash_ctrl_regs.h"

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
};

static_assert(FLASH_CTRL_PARAM_REG_NUM_BANKS == 2, "Flash must have 2 banks");

/**
 * Bootstrap states.
 *
 * OpenTitan bootstrap consists of three states between which the chip
 * transitions sequentially.
 *
 * Encoding generated with
 * $ ./util/design/sparse-fsm-encode.py -d 5 -m 3 -n 32 \
 *     -s 375382971 --language=c
 *
 * Minimum Hamming distance: 17
 * Maximum Hamming distance: 19
 * Minimum Hamming weight: 16
 * Maximum Hamming weight: 19
 */
typedef enum bootstrap_state {
  /**
   * Initial bootstrap state where the chip waits for a SECTOR_ERASE or
   * CHIP_ERASE command.
   */
  kBootstrapStateErase = 0xd4576543,
  /**
   * Second bootstrap state where the chip verifies that all data banks have
   * been erased.
   */
  kBootstrapStateEraseVerify = 0xf3c71bac,
  /**
   * Final bootstrap state. This is the main program loop where the chip handles
   * erase, program, and reset commands.
   */
  kBootstrapStateProgram = 0xbdd8ca60,
} bootstrap_state_t;

/**
 * A context object for the bootstrap state machine.
 */
typedef struct bootstrap_ctx {
  bootstrap_state_t state;
  hardened_bool_t bounds_checks_on;
} bootstrap_ctx_t;

/**
 * Handles access permissions and erases both data banks of the embedded flash.
 *
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
static rom_error_t bootstrap_chip_erase(hardened_bool_t bounds_checks_on) {
  if (bounds_checks_on == kHardenedBoolTrue) {
    // Bounds checks are defined at the granularity of pages, so we check
    // whether individual pages are safe to erase rather than erasing entire
    // banks at a time.
    flash_ctrl_bank_erase_perms_set(kHardenedBoolTrue);
    rom_error_t last_err = kErrorOk;
    for (uint32_t i = 0; i < kNumPages; ++i) {
      const uint32_t addr = i * FLASH_CTRL_PARAM_BYTES_PER_PAGE;
      const hardened_bool_t is_addr_ok =
          bootstrap_bounds_check(kSpiDeviceOpcodeChipErase, addr);
      if (is_addr_ok != kHardenedBoolTrue) {
        continue;
      }
      HARDENED_CHECK_EQ(is_addr_ok, kHardenedBoolTrue);
      rom_error_t err = flash_ctrl_data_erase(addr, kFlashCtrlEraseTypePage);
      if (err != kErrorOk) {
        last_err = err;
      }
    }
    flash_ctrl_bank_erase_perms_set(kHardenedBoolFalse);
    HARDENED_RETURN_IF_ERROR(last_err);
    return kErrorOk;
  }

  HARDENED_CHECK_EQ(bounds_checks_on, kHardenedBoolFalse);

  flash_ctrl_bank_erase_perms_set(kHardenedBoolTrue);
  rom_error_t err_0 = flash_ctrl_data_erase(0, kFlashCtrlEraseTypeBank);
  rom_error_t err_1 = flash_ctrl_data_erase(FLASH_CTRL_PARAM_BYTES_PER_BANK,
                                            kFlashCtrlEraseTypeBank);
  flash_ctrl_bank_erase_perms_set(kHardenedBoolFalse);

  HARDENED_RETURN_IF_ERROR(err_0);
  return err_1;
}

/**
 * Handles access permissions and erases a 4 KiB region in the data partition of
 * the embedded flash.
 *
 * Since OpenTitan's flash page size is 2 KiB, this function erases two
 * consecutive pages.
 *
 * @param addr Address that falls within the 4 KiB region being deleted.
 * @param bounds_checks_on Determines whether to erase single pages at a time,
 * consulting `bootstrap_bounds_check()` for each page.
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
static rom_error_t bootstrap_sector_erase(uint32_t addr,
                                          hardened_bool_t bounds_checks_on) {
  static_assert(FLASH_CTRL_PARAM_BYTES_PER_PAGE == 2048,
                "Page size must be 2 KiB");
  enum {
    /**
     * Mask for truncating `addr` to the lower 4 KiB aligned address.
     */
    kPageAddrMask = ~UINT32_C(4096) + 1,
  };

  if (addr >= kMaxAddress) {
    return kErrorBootstrapEraseAddress;
  }
  addr &= kPageAddrMask;

  if (bounds_checks_on == kHardenedBoolTrue) {
    const hardened_bool_t is_addr_ok =
        bootstrap_bounds_check(kSpiDeviceOpcodeSectorErase, addr);
    if (is_addr_ok != kHardenedBoolTrue) {
      return kErrorBootstrapIllegalAddr;
    }
    HARDENED_CHECK_EQ(is_addr_ok, kHardenedBoolTrue);
  } else {
    HARDENED_CHECK_EQ(bounds_checks_on, kHardenedBoolFalse);
  }

  flash_ctrl_data_default_perms_set((flash_ctrl_perms_t){
      .read = kMultiBitBool4False,
      .write = kMultiBitBool4False,
      .erase = kMultiBitBool4True,
  });
  rom_error_t err_0 = flash_ctrl_data_erase(addr, kFlashCtrlEraseTypePage);
  rom_error_t err_1 = flash_ctrl_data_erase(
      addr + FLASH_CTRL_PARAM_BYTES_PER_PAGE, kFlashCtrlEraseTypePage);
  flash_ctrl_data_default_perms_set((flash_ctrl_perms_t){
      .read = kMultiBitBool4False,
      .write = kMultiBitBool4False,
      .erase = kMultiBitBool4False,
  });

  HARDENED_RETURN_IF_ERROR(err_0);
  return err_1;
}

/**
 * Handles access permissions and programs up to 256 bytes of flash memory
 * starting at `addr`.
 *
 * If `byte_count` is not a multiple of flash word size, it's rounded up to next
 * flash word and missing bytes in `data` are set to `0xff`.
 *
 * @param addr Address to write to, must be flash word aligned.
 * @param byte_count Number of bytes to write. Rounded up to next flash word if
 * not a multiple of flash word size. Missing bytes in `data` are set to `0xff`.
 * @param data Data to write, must be word aligned. If `byte_count` is not a
 * multiple of flash word size, `data` must have enough space until the next
 * flash word.
 * @param bounds_checks_on Determines whether to operate on single pages at a
 * time, consulting `bootstrap_bounds_check()` for each page.
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
static rom_error_t bootstrap_page_program(uint32_t addr, size_t byte_count,
                                          uint8_t *data,
                                          hardened_bool_t bounds_checks_on) {
  static_assert(__builtin_popcount(FLASH_CTRL_PARAM_BYTES_PER_WORD) == 1,
                "Bytes per flash word must be a power of two.");
  enum {
    /**
     * Mask for checking that `addr` is flash word aligned.
     */
    kFlashWordMask = FLASH_CTRL_PARAM_BYTES_PER_WORD - 1,
    /**
     * SPI flash programming page size in bytes.
     */
    kFlashProgPageSize = 256,
    /**
     * Mask for checking whether `addr` is flash programming page aligned.
     *
     * Flash programming page size is 256 bytes, writes that start at an `addr`
     * with a non-zero LSB wrap to the start of the 256 byte region.
     */
    kFlashProgPageMask = kFlashProgPageSize - 1,
  };

  if (addr & kFlashWordMask || addr >= kMaxAddress) {
    return kErrorBootstrapProgramAddress;
  }

  if (bounds_checks_on == kHardenedBoolTrue) {
    const hardened_bool_t is_addr_ok =
        bootstrap_bounds_check(kSpiDeviceOpcodePageProgram, addr);
    if (is_addr_ok != kHardenedBoolTrue) {
      return kErrorBootstrapIllegalAddr;
    }
    HARDENED_CHECK_EQ(is_addr_ok, kHardenedBoolTrue);
  } else {
    HARDENED_CHECK_EQ(bounds_checks_on, kHardenedBoolFalse);
  }

  // Round up to next flash word and fill missing bytes with `0xff`.
  size_t flash_word_misalignment = byte_count & kFlashWordMask;
  if (flash_word_misalignment > 0) {
    size_t padding_byte_count =
        FLASH_CTRL_PARAM_BYTES_PER_WORD - flash_word_misalignment;
    for (size_t i = 0; i < padding_byte_count; ++i) {
      data[byte_count++] = 0xff;
    }
  }
  size_t rem_word_count = byte_count / sizeof(uint32_t);

  flash_ctrl_data_default_perms_set((flash_ctrl_perms_t){
      .read = kMultiBitBool4False,
      .write = kMultiBitBool4True,
      .erase = kMultiBitBool4False,
  });
  // Perform two writes if the start address is not page-aligned (256 bytes).
  // Note: Address is flash-word-aligned (8 bytes) due to the check above.
  rom_error_t err_0 = kErrorOk;
  size_t prog_page_misalignment = addr & kFlashProgPageMask;
  if (prog_page_misalignment > 0) {
    size_t word_count =
        (kFlashProgPageSize - prog_page_misalignment) / sizeof(uint32_t);
    if (word_count > rem_word_count) {
      word_count = rem_word_count;
    }
    err_0 = flash_ctrl_data_write(addr, word_count, data);
    rem_word_count -= word_count;
    data += word_count * sizeof(uint32_t);
    // Wrap to the beginning of the current page since PAGE_PROGRAM modifies
    // a single page only.
    addr &= ~(uint32_t)kFlashProgPageMask;
  }
  rom_error_t err_1 = kErrorOk;
  if (rem_word_count > 0) {
    err_1 = flash_ctrl_data_write(addr, rem_word_count, data);
  }
  flash_ctrl_data_default_perms_set((flash_ctrl_perms_t){
      .read = kMultiBitBool4False,
      .write = kMultiBitBool4False,
      .erase = kMultiBitBool4False,
  });

  HARDENED_RETURN_IF_ERROR(err_0);
  return err_1;
}

/**
 * Bootstrap state 1: Wait for an erase command and erase the data
 * partition.
 *
 * This function erases both data banks of the flash regardless of the type of
 * the erase command (CHIP_ERASE or SECTOR_ERASE).
 *
 * @param[in,out] ctx State machine context.
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
static rom_error_t bootstrap_handle_erase(bootstrap_ctx_t *ctx) {
  HARDENED_CHECK_EQ(ctx->state, kBootstrapStateErase);

  spi_device_cmd_t cmd;
  RETURN_IF_ERROR(spi_device_cmd_get(&cmd));
  // Erase requires WREN, ignore if WEL is not set.
  if (!bitfield_bit32_read(spi_device_flash_status_get(), kSpiDeviceWelBit)) {
    return kErrorOk;
  }

  rom_error_t error = kErrorUnknown;
  switch (cmd.opcode) {
    case kSpiDeviceOpcodeChipErase:
    case kSpiDeviceOpcodeSectorErase:
      error = bootstrap_chip_erase(ctx->bounds_checks_on);
      HARDENED_RETURN_IF_ERROR(error);
      ctx->state = kBootstrapStateEraseVerify;
      // Note: We clear WIP and WEN bits in `bootstrap_handle_erase_verify()`
      // after checking that both data banks have been erased.
      break;
    default:
      // Ignore any other command, e.g. PAGE_PROGRAM, RESET, and clear WIP and
      // WEN bits right away.
      spi_device_flash_status_clear();
      error = kErrorOk;
  }

  return error;
}

/**
 * Bootstrap state 2: Verify that all data banks have been erased.
 *
 * This function also clears the WIP and WEN bits of the flash status register.
 *
 * @param[in,out] ctx State machine context.
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
static rom_error_t bootstrap_handle_erase_verify(bootstrap_ctx_t *ctx) {
  HARDENED_CHECK_EQ(ctx->state, kBootstrapStateEraseVerify);

  if (ctx->bounds_checks_on == kHardenedBoolTrue) {
    rom_error_t last_err = kErrorOk;
    for (uint32_t i = 0; i < kNumPages; ++i) {
      const uint32_t addr = i * FLASH_CTRL_PARAM_BYTES_PER_PAGE;
      const hardened_bool_t is_addr_ok =
          bootstrap_bounds_check(kSpiDeviceOpcodeChipErase, addr);
      if (is_addr_ok != kHardenedBoolTrue) {
        continue;
      }
      HARDENED_CHECK_EQ(is_addr_ok, kHardenedBoolTrue);
      rom_error_t err =
          flash_ctrl_data_erase_verify(addr, kFlashCtrlEraseTypePage);
      if (err != kErrorOk) {
        last_err = err;
      }
    }

    ctx->state = kBootstrapStateProgram;
    spi_device_flash_status_clear();

    HARDENED_RETURN_IF_ERROR(last_err);
    return last_err;
  }

  HARDENED_CHECK_EQ(ctx->bounds_checks_on, kHardenedBoolFalse);

  rom_error_t err_0 = flash_ctrl_data_erase_verify(0, kFlashCtrlEraseTypeBank);
  rom_error_t err_1 = flash_ctrl_data_erase_verify(
      FLASH_CTRL_PARAM_BYTES_PER_BANK, kFlashCtrlEraseTypeBank);
  HARDENED_RETURN_IF_ERROR(err_0);
  HARDENED_RETURN_IF_ERROR(err_1);

  ctx->state = kBootstrapStateProgram;
  spi_device_flash_status_clear();
  return err_0;
}

/**
 * Bootstrap state 3: (Erase/)Program loop.
 *
 * @param[in,out] ctx State machine context.
 * @return Result of the operation.
 */
OT_WARN_UNUSED_RESULT
static rom_error_t bootstrap_handle_program(bootstrap_ctx_t *ctx) {
  static_assert(alignof(spi_device_cmd_t) >= sizeof(uint32_t) &&
                    offsetof(spi_device_cmd_t, payload) >= sizeof(uint32_t),
                "Payload must be word aligned.");
  static_assert(
      sizeof((spi_device_cmd_t){0}.payload) % FLASH_CTRL_PARAM_BYTES_PER_WORD ==
          0,
      "Payload size must be a multiple of flash word size.");

  HARDENED_CHECK_EQ(ctx->state, kBootstrapStateProgram);

  spi_device_cmd_t cmd;
  RETURN_IF_ERROR(spi_device_cmd_get(&cmd));
  // Erase and program require WREN, ignore if WEL is not set.
  if (cmd.opcode != kSpiDeviceOpcodeReset &&
      !bitfield_bit32_read(spi_device_flash_status_get(), kSpiDeviceWelBit)) {
    return kErrorOk;
  }

  rom_error_t error = kErrorUnknown;
  switch (cmd.opcode) {
    case kSpiDeviceOpcodeChipErase:
      error = bootstrap_chip_erase(ctx->bounds_checks_on);
      break;
    case kSpiDeviceOpcodeSectorErase:
      error = bootstrap_sector_erase(cmd.address, ctx->bounds_checks_on);
      break;
    case kSpiDeviceOpcodePageProgram:
      error = bootstrap_page_program(cmd.address, cmd.payload_byte_count,
                                     cmd.payload, ctx->bounds_checks_on);
      break;
    case kSpiDeviceOpcodeReset:
      rstmgr_reset();
#ifdef OT_PLATFORM_RV32
      HARDENED_TRAP();
#else
      // If this is an off-target test, return `kErrorUnknown` to be able to
      // test without requiring EXPECT_DEATH.
      error = kErrorUnknown;
#endif
      break;
    default:
      // We don't expect any other commands but we can potentially end up
      // here with a 0x0 opcode due to glitches on SPI or strap lines (see
      // #11871).
      error = kErrorOk;
  }
  HARDENED_RETURN_IF_ERROR(error);

  spi_device_flash_status_clear();
  return error;
}

rom_error_t enter_bootstrap(hardened_bool_t bounds_checks_on) {
  spi_device_init();

  // Bootstrap event loop.
  bootstrap_ctx_t ctx = {
      .state = kBootstrapStateErase,
      .bounds_checks_on = bounds_checks_on,
  };

  rom_error_t error = kErrorUnknown;
  while (true) {
    switch (launder32(ctx.state)) {
      case kBootstrapStateErase:
        HARDENED_CHECK_EQ(ctx.state, kBootstrapStateErase);
        error = bootstrap_handle_erase(&ctx);
        break;
      case kBootstrapStateEraseVerify:
        HARDENED_CHECK_EQ(ctx.state, kBootstrapStateEraseVerify);
        error = bootstrap_handle_erase_verify(&ctx);
        break;
      case kBootstrapStateProgram:
        HARDENED_CHECK_EQ(ctx.state, kBootstrapStateProgram);
        error = bootstrap_handle_program(&ctx);
        break;
      default:
        error = kErrorBootstrapInvalidState;
    }
    HARDENED_RETURN_IF_ERROR(error);
  }
  HARDENED_TRAP();
}
