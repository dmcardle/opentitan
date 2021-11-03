// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#ifndef OPENTITAN_SW_DEVICE_LIB_DIF_AUTOGEN_DIF_SPI_DEVICE_AUTOGEN_H_
#define OPENTITAN_SW_DEVICE_LIB_DIF_AUTOGEN_DIF_SPI_DEVICE_AUTOGEN_H_

// THIS FILE HAS BEEN GENERATED, DO NOT EDIT MANUALLY. COMMAND:
// util/make_new_dif.py --mode=regen --only=autogen

/**
 * @file
 * @brief <a href="/hw/ip/spi_device/doc/">SPI_DEVICE</a> Device Interface
 * Functions
 */

#include <stdbool.h>
#include <stdint.h>

#include "sw/device/lib/base/macros.h"
#include "sw/device/lib/base/mmio.h"
#include "sw/device/lib/dif/dif_base.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/**
 * A handle to spi_device.
 *
 * This type should be treated as opaque by users.
 */
typedef struct dif_spi_device {
  /**
   * The base address for the spi_device hardware registers.
   */
  mmio_region_t base_addr;
} dif_spi_device_t;

/**
 * Creates a new handle for a(n) spi_device peripheral.
 *
 * This function does not actuate the hardware.
 *
 * @param base_addr The MMIO base address of the spi_device peripheral.
 * @param[out] spi_device Out param for the initialized handle.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_init(mmio_region_t base_addr,
                                 dif_spi_device_t *spi_device);

/**
 * A spi_device alert type.
 */
typedef enum dif_spi_device_alert {
  /**
   * This fatal alert is triggered when a fatal TL-UL bus integrity fault is
   * detected.
   */
  kDifSpiDeviceAlertFatalFault = 0,
} dif_spi_device_alert_t;

/**
 * Forces a particular alert, causing it to be escalated as if the hardware
 * had raised it.
 *
 * @param spi_device A spi_device handle.
 * @param alert The alert to force.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_alert_force(const dif_spi_device_t *spi_device,
                                        dif_spi_device_alert_t alert);

/**
 * A spi_device interrupt request type.
 */
typedef enum dif_spi_device_irq {
  /**
   * RX SRAM FIFO Full
   */
  kDifSpiDeviceIrqRxFull = 0,
  /**
   * RX SRAM FIFO is above the level
   */
  kDifSpiDeviceIrqRxWatermark = 1,
  /**
   * TX SRAM FIFO is under the level
   */
  kDifSpiDeviceIrqTxWatermark = 2,
  /**
   * SDI in FwMode has error
   */
  kDifSpiDeviceIrqRxError = 3,
  /**
   * RX Async FIFO overflow
   */
  kDifSpiDeviceIrqRxOverflow = 4,
  /**
   * TX Async FIFO underflow
   */
  kDifSpiDeviceIrqTxUnderflow = 5,
  /**
   * TPM Header(Command/Address) buffer available
   */
  kDifSpiDeviceIrqTpmHeaderNotEmpty = 6,
} dif_spi_device_irq_t;

/**
 * A snapshot of the state of the interrupts for this IP.
 *
 * This is an opaque type, to be used with the `dif_spi_device_irq_get_state()`
 * function.
 */
typedef uint32_t dif_spi_device_irq_state_snapshot_t;

/**
 * Returns the state of all interrupts (i.e., pending or not) for this IP.
 *
 * @param spi_device A spi_device handle.
 * @param[out] snapshot Out-param for interrupt state snapshot.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_get_state(
    const dif_spi_device_t *spi_device,
    dif_spi_device_irq_state_snapshot_t *snapshot);

/**
 * Returns whether a particular interrupt is currently pending.
 *
 * @param spi_device A spi_device handle.
 * @param irq An interrupt request.
 * @param[out] is_pending Out-param for whether the interrupt is pending.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_is_pending(const dif_spi_device_t *spi_device,
                                           dif_spi_device_irq_t irq,
                                           bool *is_pending);

/**
 * Acknowledges all interrupts, indicating to the hardware that all
 * interrupts have been successfully serviced.
 *
 * @param spi_device A spi_device handle.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_acknowledge_all(
    const dif_spi_device_t *spi_device);

/**
 * Acknowledges a particular interrupt, indicating to the hardware that it has
 * been successfully serviced.
 *
 * @param spi_device A spi_device handle.
 * @param irq An interrupt request.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_acknowledge(const dif_spi_device_t *spi_device,
                                            dif_spi_device_irq_t irq);

/**
 * Forces a particular interrupt, causing it to be serviced as if hardware had
 * asserted it.
 *
 * @param spi_device A spi_device handle.
 * @param irq An interrupt request.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_force(const dif_spi_device_t *spi_device,
                                      dif_spi_device_irq_t irq);

/**
 * A snapshot of the enablement state of the interrupts for this IP.
 *
 * This is an opaque type, to be used with the
 * `dif_spi_device_irq_disable_all()` and `dif_spi_device_irq_restore_all()`
 * functions.
 */
typedef uint32_t dif_spi_device_irq_enable_snapshot_t;

/**
 * Checks whether a particular interrupt is currently enabled or disabled.
 *
 * @param spi_device A spi_device handle.
 * @param irq An interrupt request.
 * @param[out] state Out-param toggle state of the interrupt.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_get_enabled(const dif_spi_device_t *spi_device,
                                            dif_spi_device_irq_t irq,
                                            dif_toggle_t *state);

/**
 * Sets whether a particular interrupt is currently enabled or disabled.
 *
 * @param spi_device A spi_device handle.
 * @param irq An interrupt request.
 * @param state The new toggle state for the interrupt.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_set_enabled(const dif_spi_device_t *spi_device,
                                            dif_spi_device_irq_t irq,
                                            dif_toggle_t state);

/**
 * Disables all interrupts, optionally snapshotting all enable states for later
 * restoration.
 *
 * @param spi_device A spi_device handle.
 * @param[out] snapshot Out-param for the snapshot; may be `NULL`.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_disable_all(
    const dif_spi_device_t *spi_device,
    dif_spi_device_irq_enable_snapshot_t *snapshot);

/**
 * Restores interrupts from the given (enable) snapshot.
 *
 * @param spi_device A spi_device handle.
 * @param snapshot A snapshot to restore from.
 * @return The result of the operation.
 */
OT_WARN_UNUSED_RESULT
dif_result_t dif_spi_device_irq_restore_all(
    const dif_spi_device_t *spi_device,
    const dif_spi_device_irq_enable_snapshot_t *snapshot);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // OPENTITAN_SW_DEVICE_LIB_DIF_AUTOGEN_DIF_SPI_DEVICE_AUTOGEN_H_
