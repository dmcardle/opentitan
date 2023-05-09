// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdint.h>

#include "sw/device/lib/arch/device.h"
#include "sw/device/lib/dif/dif_flash_ctrl.h"
#include "sw/device/lib/dif/dif_lc_ctrl.h"
#include "sw/device/lib/dif/dif_uart.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/runtime/print.h"
#include "sw/device/lib/testing/pinmux_testutils.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/silicon_creator/manuf/lib/isolated_flash_partition.h"

#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"

static dif_uart_t uart;
static dif_pinmux_t pinmux;
static dif_flash_ctrl_state_t flash_ctrl_state;
static dif_lc_ctrl_t lc_ctrl;

/**
 * Initializes all DIF handles used in this SRAM program.
 */
static status_t peripheral_handles_init(void) {
  TRY(dif_flash_ctrl_init_state(
      &flash_ctrl_state,
      mmio_region_from_addr(TOP_EARLGREY_FLASH_CTRL_CORE_BASE_ADDR)));
  TRY(dif_lc_ctrl_init(mmio_region_from_addr(TOP_EARLGREY_LC_CTRL_BASE_ADDR),
                       &lc_ctrl));
  TRY(dif_pinmux_init(mmio_region_from_addr(TOP_EARLGREY_PINMUX_AON_BASE_ADDR),
                      &pinmux));
  TRY(dif_uart_init(mmio_region_from_addr(TOP_EARLGREY_UART0_BASE_ADDR),
                    &uart));
  return OK_STATUS();
}

void sram_main(void) {
  CHECK_STATUS_OK(peripheral_handles_init());

  // Initialize UART (for console, since we do not have the OTTF).
  pinmux_testutils_init(&pinmux);

    CHECK(kUartBaudrate <= UINT32_MAX, "kUartBaudrate must fit in uint32_t");
  CHECK(kClockFreqPeripheralHz <= UINT32_MAX,
        "kClockFreqPeripheralHz must fit in uint32_t");
  CHECK_DIF_OK(
      dif_uart_configure(&uart, (dif_uart_config_t){
                                    .baudrate = (uint32_t)kUartBaudrate,
                                    .clk_freq_hz = (uint32_t)kClockFreqPeripheralHz,
                                    .parity_enable = kDifToggleDisabled,
                                    .parity = kDifUartParityEven,
                                    .tx_enable = kDifToggleEnabled,
                                    .rx_enable = kDifToggleEnabled,
                                }));
  base_uart_stdout(&uart);

  // Generated expected wafer authentication secret to write to the flash
  // isolated partition in this test.
  uint32_t actual_wafer_auth_secret[kWaferAuthSecretSizeInWords] = {0};
  uint32_t expected_wafer_auth_secret[kWaferAuthSecretSizeInWords];
  for (size_t i = 0; i < kWaferAuthSecretSizeInWords; ++i) {
    expected_wafer_auth_secret[i] = 0xdeadbeef;
  }

  // Read LC state.
  dif_lc_ctrl_state_t lc_state = kDifLcCtrlStateInvalid;
  CHECK_DIF_OK(dif_lc_ctrl_get_state(&lc_ctrl, &lc_state));

  switch (lc_state) {
    case kDifLcCtrlStateTestUnlocked0:
    case kDifLcCtrlStateTestUnlocked1:
    case kDifLcCtrlStateTestUnlocked2:
    case kDifLcCtrlStateTestUnlocked3:
    case kDifLcCtrlStateTestUnlocked4:
    case kDifLcCtrlStateTestUnlocked5:
    case kDifLcCtrlStateTestUnlocked6:
    case kDifLcCtrlStateTestUnlocked7:
      LOG_INFO("Writing to the isolated flash partition.");
      CHECK_STATUS_OK(isolated_flash_partition_write(
          &flash_ctrl_state, expected_wafer_auth_secret,
          kWaferAuthSecretSizeInWords));
      LOG_INFO("Attempting to read back what was written.");
      CHECK_STATUS_NOT_OK(isolated_flash_partition_read(
          &flash_ctrl_state, kWaferAuthSecretSizeInWords,
          actual_wafer_auth_secret));
      // TODO: perform LC transition to DEV, PROD, or PROD_END.
      LOG_INFO("Done.");
      break;
    case kDifLcCtrlStateDev:
    case kDifLcCtrlStateProd:
    case kDifLcCtrlStateProdEnd:
      LOG_INFO("Reading the isolated flash partition.");
      CHECK_STATUS_OK(isolated_flash_partition_read(&flash_ctrl_state,
                                                    kWaferAuthSecretSizeInWords,
                                                    actual_wafer_auth_secret));
      // TODO: check wafer authentication secret is what is expected.
      LOG_INFO("Done.");
      break;
    default:
      test_status_set(kTestStatusFailed);
      break;
  }

  test_status_set(kTestStatusPassed);
}
