// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "sw/device/silicon_creator/rom_ext/bootstrap.h"

#include <vector>

#include "absl/types/span.h"
#include "gtest/gtest.h"
#include "sw/device/lib/base/hardened.h"
#include "sw/device/silicon_creator/lib/bootstrap.h"
#include "sw/device/silicon_creator/lib/bootstrap_unittest_util.h"
#include "sw/device/silicon_creator/lib/drivers/mock_flash_ctrl.h"
#include "sw/device/silicon_creator/lib/drivers/mock_otp.h"
#include "sw/device/silicon_creator/lib/drivers/mock_rstmgr.h"
#include "sw/device/silicon_creator/lib/drivers/mock_spi_device.h"
#include "sw/device/silicon_creator/lib/error.h"
#include "sw/device/silicon_creator/lib/error_unittest_util.h"
#include "sw/device/silicon_creator/testing/rom_test.h"

#include "flash_ctrl_regs.h"
#include "hw/ip/otp_ctrl/data/otp_ctrl_regs.h"

namespace rom_ext_bootstrap_unittest {
namespace {

using ::testing::AtLeast;
using ::testing::DoAll;
using ::testing::Each;
using ::testing::Eq;
using ::testing::NotNull;
using ::testing::Return;
using ::testing::SetArgPointee;

using bootstrap_unittest_util::ChipEraseCmd;
using bootstrap_unittest_util::PageProgramCmd;
using bootstrap_unittest_util::ResetCmd;
using bootstrap_unittest_util::SectorEraseCmd;

/**
 * A helper class for managing the offset-based endpoints of flash regions. It's
 * conceptually similar to a span, but it does not hold any pointers.
 */
class OffsetRegion {
 public:
  OffsetRegion(uint32_t start, uint32_t len) : start_(start), len_(len) {
    EXPECT_GE(len, 0);
  }
  OffsetRegion() = delete;
  OffsetRegion(const OffsetRegion &) = delete;
  OffsetRegion(OffsetRegion &&) = default;

  uint32_t start() const { return start_; }
  uint32_t len() const { return len_; }

  /**
   * The offset of the last element in this region. This is *not* a past-the-end
   * pointer like `std::end()`.
   */
  uint32_t Last() const { return start_ + len_ - 1; }
  /**
   * Create a new region, pushed to the right by `amount` bytes.
   */
  OffsetRegion ShiftRight(uint32_t amount) const {
    return OffsetRegion(start_ + amount, len_);
  }

 private:
  uint32_t start_ = 0;
  uint32_t len_ = 0;
};

/**
 * A collection of functions for simulating flash_ctrl operations on a chunk
 * of host memory.
 */
class FlashCtrlSim {
 public:
  enum FlashByte : char {
    kDefault = 0x11,
    kErased = 0x22,
    kErasedVerified = 0x33,
  };

  FlashCtrlSim() : memory_(flash_size(), FlashByte::kDefault) {}

  rom_error_t DataErase(uint32_t addr, flash_ctrl_erase_type_t erase_type) {
    using ::testing::Each;
    using ::testing::Eq;

    absl::Span<char> region;
    switch (erase_type) {
      case kFlashCtrlEraseTypeBank: {
        EXPECT_EQ(addr % bank_size(), 0);
        region = GetFlash().subspan(addr, bank_size());
        break;
      }
      case kFlashCtrlEraseTypePage: {
        EXPECT_EQ(addr % page_size(), 0);
        region = GetFlash().subspan(addr, page_size());
        break;
      }
      default:
        ADD_FAILURE() << "DataErase is only implemented for banks and pages";
        return kErrorUnknown;
    }

    EXPECT_THAT(region, Each(Eq(FlashByte::kDefault)))
        << "DataErase should only see memory that hasn't been touched yet";
    memset(region.data(), FlashByte::kErased, region.size());

    return kErrorOk;
  }

  rom_error_t DataEraseVerify(uint32_t addr,
                              flash_ctrl_erase_type_t erase_type) {
    using ::testing::Each;
    using ::testing::Eq;

    absl::Span<char> region;
    switch (erase_type) {
      case kFlashCtrlEraseTypeBank: {
        EXPECT_EQ(addr % bank_size(), 0);
        region = GetFlash().subspan(addr, bank_size());
        break;
      }
      case kFlashCtrlEraseTypePage: {
        EXPECT_EQ(addr % page_size(), 0);
        region = GetFlash().subspan(addr, page_size());
        break;
      }
      default:
        ADD_FAILURE()
            << "DataEraseVerify is only implemented for banks and pages";
        return kErrorUnknown;
    }

    EXPECT_THAT(region, Each(Eq(FlashByte::kErased)))
        << "DataEraseVerify should only encounter that is already erased";
    memset(region.data(), FlashByte::kErasedVerified, region.size());
    return kErrorOk;
  }

  static constexpr size_t flash_size() { return num_banks() * bank_size(); }

  absl::Span<char> GetFlash() { return absl::MakeSpan(memory_); }
  absl::Span<char> GetSlotA() { return GetFlash().subspan(0, bank_size()); }
  absl::Span<char> GetSlotB() { return GetFlash().subspan(bank_size()); }

  absl::Span<char> GetRomExtSlotA() {
    return GetSlotA().subspan(0, rom_ext_size());
  }

  absl::Span<char> GetRomExtSlotB() {
    return GetSlotB().subspan(0, rom_ext_size());
  }

 private:
  static constexpr size_t num_banks() { return FLASH_CTRL_PARAM_REG_NUM_BANKS; }
  static constexpr size_t bank_size() {
    return FLASH_CTRL_PARAM_BYTES_PER_BANK;
  }
  static constexpr size_t page_size() {
    return FLASH_CTRL_PARAM_BYTES_PER_PAGE;
  }
  static constexpr size_t rom_ext_size() { return CHIP_ROM_EXT_SIZE_MAX; }

  std::vector<char> memory_;
};

/**
 * A test fixture with convenience methods for setting expectations related to
 * ROM_EXT bootstrap.
 */
class RomExtBootstrapTest : public bootstrap_unittest_util::BootstrapTest {
 protected:
  /**
   * Sets an expectation that `otp_read32()` will be called with the address
   * of ROM_EXT_BOOTSTRAP_EN.
   *
   * @param value The value to return from the mocked `otp_read32()`.
   */
  void SetRomExtBootstrapEnabledInOtp(uint32_t value) {
    EXPECT_CALL(otp_,
                read32(OTP_CTRL_PARAM_OWNER_SW_CFG_ROM_EXT_BOOTSTRAP_EN_OFFSET))
        .WillOnce(::testing::Return(value));
  }

  /**
   * Sets an expectation that `rstmgr_reason_get()` will be called.
   *
   * @param value The value to return from the mocked `rstmgr_reason_get()`.
   */
  void SetResetReason(uint32_t value) {
    EXPECT_CALL(rstmgr_, ReasonGet()).WillOnce(::testing::Return(value));
  }

  /**
   * Delegate some flash_ctrl operations to `FlashCtrlSim`.
   */
  void DelegateToFlashCtrlSim() {
    using ::testing::_;

    ON_CALL(flash_ctrl_, DataErase(_, _))
        .WillByDefault([&](uint32_t addr, flash_ctrl_erase_type_t type) {
          return flash_ctrl_sim_.DataErase(addr, type);
        });
    ON_CALL(flash_ctrl_, DataEraseVerify(_, _))
        .WillByDefault([&](uint32_t addr, flash_ctrl_erase_type_t type) {
          return flash_ctrl_sim_.DataEraseVerify(addr, type);
        });
  }

  void ExpectRomExtSlotA(FlashCtrlSim::FlashByte byte) {
    const absl::Span<char> rom_ext_slot_a = flash_ctrl_sim_.GetRomExtSlotA();
    EXPECT_FALSE(rom_ext_slot_a.empty());
    EXPECT_THAT(rom_ext_slot_a, Each(Eq(byte)));
  }

  void ExpectRomExtSlotB(FlashCtrlSim::FlashByte byte) {
    const absl::Span<char> rom_ext_slot_b = flash_ctrl_sim_.GetRomExtSlotB();
    EXPECT_FALSE(rom_ext_slot_b.empty());
    EXPECT_THAT(rom_ext_slot_b, Each(Eq(byte)));
  }

  void ExpectSuffixSlotA(FlashCtrlSim::FlashByte byte) {
    const absl::Span<char> slot_a_suffix = flash_ctrl_sim_.GetSlotA().subspan(
        flash_ctrl_sim_.GetRomExtSlotA().size());
    EXPECT_FALSE(slot_a_suffix.empty());
    EXPECT_THAT(slot_a_suffix, Each(Eq(byte)));
  }

  void ExpectSuffixSlotB(FlashCtrlSim::FlashByte byte) {
    const absl::Span<char> slot_b_suffix = flash_ctrl_sim_.GetSlotB().subspan(
        flash_ctrl_sim_.GetRomExtSlotB().size());
    EXPECT_THAT(slot_b_suffix, Each(Eq(byte)));
  }

  FlashCtrlSim flash_ctrl_sim_;
};

TEST_F(RomExtBootstrapTest, BootstrapDisabledByOtp) {
  // Bootstrap is disabled when the OTP value is hardened false.
  SetRomExtBootstrapEnabledInOtp(kHardenedBoolFalse);
  EXPECT_EQ(rom_ext_bootstrap_enabled(), kHardenedBoolFalse);

  // A hardened boolean that is neither true nor false.
  constexpr hardened_bool_t kHardenedBoolMiddle =
      static_cast<hardened_bool_t>(kHardenedBoolFalse + 1);
  static_assert(
      kHardenedBoolMiddle != kHardenedBoolTrue,
      "The value of kHardenedBoolMiddle collided with kHardenedBoolTrue");
  static_assert(
      kHardenedBoolMiddle != kHardenedBoolFalse,
      "The value of kHardenedBoolMiddle collided with kHardenedBoolFalse");

  // Bootstrap is disabled when the OTP value is an invalid hardened boolean.
  SetRomExtBootstrapEnabledInOtp(kHardenedBoolMiddle);
  EXPECT_EQ(rom_ext_bootstrap_enabled(), kHardenedBoolFalse);
}

TEST_F(RomExtBootstrapTest, BootstrapDisabledByResetReason) {
  // Bootstrap is disabled when the OTP value is hardened true, but the reset
  // reason is zero.
  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(0);
  EXPECT_EQ(rom_ext_bootstrap_enabled(), kHardenedBoolFalse);

  // Bootstrap is disabled when the OTP value is hardened true, but the reset
  // reason is something other than PoR
  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonLowPowerExit);
  EXPECT_EQ(rom_ext_bootstrap_enabled(), kHardenedBoolFalse);
}

TEST_F(RomExtBootstrapTest, BootstrapEnabled) {
  // Bootstrap is enabled when the OTP value is hardened true and the reset
  // reason is PoR.
  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn);
  EXPECT_EQ(rom_ext_bootstrap_enabled(), kHardenedBoolTrue);

  // Bootstrap is enabled when the OTP value is hardened true and the reset
  // reason contains PoR and other values.
  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn | 1 << kRstmgrReasonSoftwareRequest);
  EXPECT_EQ(rom_ext_bootstrap_enabled(), kHardenedBoolTrue);
}

// Verify that `rom_ext_bootstrap()` fails with the appropriate status when
// bootstrap is disabled in OTP.
TEST_F(RomExtBootstrapTest, TryBootstrapDisabled) {
  SetRomExtBootstrapEnabledInOtp(kHardenedBoolFalse);
  EXPECT_EQ(rom_ext_bootstrap(), kErrorBootstrapDisabledRomExt);
}

TEST_F(RomExtBootstrapTest, TryBootstrapEnabledSimple) {
  using FlashByte = FlashCtrlSim::FlashByte;

  // This test will forward calls to flash_ctrl functions to
  // `flash_ctrl_sim_`, enabling us to set expectations in terms of the
  // contents of the flash.
  DelegateToFlashCtrlSim();

  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn | 1 << kRstmgrReasonSoftwareRequest);

  EXPECT_CALL(spi_device_, Init());

  // bootstrap_handle_erase
  ExpectSpiCmd(ChipEraseCmd());
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolTrue));
  EXPECT_CALL(flash_ctrl_, DataErase(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolFalse));

  // bootstrap_handle_erase_verify
  EXPECT_CALL(flash_ctrl_, DataEraseVerify(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // bootstrap_handle_program
  ExpectSpiCmd(ResetCmd());
  EXPECT_CALL(rstmgr_, Reset());

  EXPECT_THAT(flash_ctrl_sim_.GetFlash(), Each(Eq(FlashByte::kDefault)))
      << "Before rom_ext_bootstrap(), flash should be unmodified.";

  // Host-specific behavior causes bootstrap to return `kErrorUnknown` on RESET.
  EXPECT_EQ(rom_ext_bootstrap(), kErrorUnknown);

  ExpectRomExtSlotA(FlashByte::kDefault);
  ExpectSuffixSlotA(FlashByte::kErasedVerified);
  ExpectRomExtSlotB(FlashByte::kDefault);
  ExpectSuffixSlotB(FlashByte::kErasedVerified);
}

TEST_F(RomExtBootstrapTest, TryBootstrapEnabledJunkBeforeEraseCmd) {
  using FlashByte = FlashCtrlSim::FlashByte;

  // This test will forward calls to flash_ctrl functions to
  // `flash_ctrl_sim_`, enabling us to set expectations in terms of the
  // contents of the flash.
  DelegateToFlashCtrlSim();

  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn | 1 << kRstmgrReasonSoftwareRequest);

  EXPECT_CALL(spi_device_, Init());

  // bootstrap_handle_erase

  // Non-erase command PAGE_PROGRAM is ignored.
  ExpectSpiCmd(PageProgramCmd(0x0, 123));
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // Non-erase command RESET is ignored.
  ExpectSpiCmd(ResetCmd());
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // CHIP_ERASE command kicks off the bootstrap procedure.
  ExpectSpiCmd(ChipEraseCmd());
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolTrue));
  EXPECT_CALL(flash_ctrl_, DataErase(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolFalse));

  // bootstrap_handle_erase_verify
  EXPECT_CALL(flash_ctrl_, DataEraseVerify(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // bootstrap_handle_program
  ExpectSpiCmd(ResetCmd());
  EXPECT_CALL(rstmgr_, Reset());

  EXPECT_THAT(flash_ctrl_sim_.GetFlash(), Each(Eq(FlashByte::kDefault)))
      << "Before rom_ext_bootstrap(), flash should be unmodified.";

  // Host-specific behavior causes bootstrap to return `kErrorUnknown` on RESET.
  EXPECT_EQ(rom_ext_bootstrap(), kErrorUnknown);

  ExpectRomExtSlotA(FlashByte::kDefault);
  ExpectSuffixSlotA(FlashByte::kErasedVerified);
  ExpectRomExtSlotB(FlashByte::kDefault);
  ExpectSuffixSlotB(FlashByte::kErasedVerified);
}

// This test demonstrates that bootstrap mode will refuse to act on SPI
// commands that would erase any part of ROM_EXT in slot A.
TEST_F(RomExtBootstrapTest, BootstrapProtectsRomExtWithSectorEraseInSlotA) {
  using FlashByte = FlashCtrlSim::FlashByte;

  // This test will forward calls to flash_ctrl functions to
  // `flash_ctrl_sim_`, enabling us to set expectations in terms of the
  // contents of the flash.
  DelegateToFlashCtrlSim();

  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn | 1 << kRstmgrReasonSoftwareRequest);

  EXPECT_CALL(spi_device_, Init());

  // bootstrap_handle_erase

  // CHIP_ERASE command kicks off the bootstrap procedure.
  ExpectSpiCmd(ChipEraseCmd());
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolTrue));
  EXPECT_CALL(flash_ctrl_, DataErase(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolFalse));

  // bootstrap_handle_erase_verify
  EXPECT_CALL(flash_ctrl_, DataEraseVerify(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // bootstrap_handle_program
  ExpectSpiCmd(SectorEraseCmd(0x0));
  ExpectSpiFlashStatusGet(true);

  EXPECT_THAT(flash_ctrl_sim_.GetFlash(), Each(Eq(FlashByte::kDefault)))
      << "Before rom_ext_bootstrap(), flash should be unmodified.";

  EXPECT_EQ(rom_ext_bootstrap(), kErrorBootstrapIllegalAddr);

  ExpectRomExtSlotA(FlashByte::kDefault);
  ExpectSuffixSlotA(FlashByte::kErasedVerified);
  ExpectRomExtSlotB(FlashByte::kDefault);
  ExpectSuffixSlotB(FlashByte::kErasedVerified);
}

// This test demonstrates that bootstrap mode will refuse to act on SPI
// commands that would erase any part of ROM_EXT in slot B.
TEST_F(RomExtBootstrapTest, BootstrapProtectsRomExtWithSectorEraseInSlotB) {
  using FlashByte = FlashCtrlSim::FlashByte;

  // This test will forward calls to flash_ctrl functions to
  // `flash_ctrl_sim_`, enabling us to set expectations in terms of the
  // contents of the flash.
  DelegateToFlashCtrlSim();

  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn | 1 << kRstmgrReasonSoftwareRequest);

  EXPECT_CALL(spi_device_, Init());

  // bootstrap_handle_erase

  // CHIP_ERASE command kicks off the bootstrap procedure.
  ExpectSpiCmd(ChipEraseCmd());
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolTrue));
  EXPECT_CALL(flash_ctrl_, DataErase(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolFalse));

  // bootstrap_handle_erase_verify
  EXPECT_CALL(flash_ctrl_, DataEraseVerify(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // bootstrap_handle_program
  ExpectSpiCmd(SectorEraseCmd(FLASH_CTRL_PARAM_BYTES_PER_BANK +
                              CHIP_ROM_EXT_SIZE_MAX / 2));
  ExpectSpiFlashStatusGet(true);

  EXPECT_THAT(flash_ctrl_sim_.GetFlash(), Each(Eq(FlashByte::kDefault)))
      << "Before rom_ext_bootstrap(), flash should be unmodified.";

  EXPECT_EQ(rom_ext_bootstrap(), kErrorBootstrapIllegalAddr);

  ExpectRomExtSlotA(FlashByte::kDefault);
  ExpectSuffixSlotA(FlashByte::kErasedVerified);
  ExpectRomExtSlotB(FlashByte::kDefault);
  ExpectSuffixSlotB(FlashByte::kErasedVerified);
}

// Bootstrap and send a PAGE_PROGRAM command that targets an address inside
// ROM_EXT.
TEST_F(RomExtBootstrapTest, BootstrapProtectsRomExtWithPageProgramInSlotA) {
  using FlashByte = FlashCtrlSim::FlashByte;

  // This test will forward calls to flash_ctrl functions to
  // `flash_ctrl_sim_`, enabling us to set expectations in terms of the
  // contents of the flash.
  DelegateToFlashCtrlSim();

  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn | 1 << kRstmgrReasonSoftwareRequest);

  EXPECT_CALL(spi_device_, Init());

  // bootstrap_handle_erase
  ExpectSpiCmd(ChipEraseCmd());
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolTrue));
  EXPECT_CALL(flash_ctrl_, DataErase(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolFalse));

  // bootstrap_handle_erase_verify
  EXPECT_CALL(flash_ctrl_, DataEraseVerify(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // bootstrap_handle_program
  ExpectSpiCmd(PageProgramCmd(CHIP_ROM_EXT_SIZE_MAX / 2, 1));
  ExpectSpiFlashStatusGet(true);

  EXPECT_THAT(flash_ctrl_sim_.GetFlash(), Each(Eq(FlashByte::kDefault)))
      << "Before rom_ext_bootstrap(), flash should be unmodified.";

  // TODO(dmcardle) Shouldn't this be kErrorOk?
  EXPECT_EQ(rom_ext_bootstrap(), kErrorBootstrapIllegalAddr);

  ExpectRomExtSlotA(FlashByte::kDefault);
  ExpectSuffixSlotA(FlashByte::kErasedVerified);
  ExpectRomExtSlotB(FlashByte::kDefault);
  ExpectSuffixSlotB(FlashByte::kErasedVerified);
}

TEST_F(RomExtBootstrapTest, BootstrapProtectsRomExtWithPageProgramInSlotB) {
  using FlashByte = FlashCtrlSim::FlashByte;

  // This test will forward calls to flash_ctrl functions to
  // `flash_ctrl_sim_`, enabling us to set expectations in terms of the
  // contents of the flash.
  DelegateToFlashCtrlSim();

  SetRomExtBootstrapEnabledInOtp(kHardenedBoolTrue);
  SetResetReason(1 << kRstmgrReasonPowerOn | 1 << kRstmgrReasonSoftwareRequest);

  EXPECT_CALL(spi_device_, Init());

  // bootstrap_handle_erase
  ExpectSpiCmd(ChipEraseCmd());
  ExpectSpiFlashStatusGet(true);
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolTrue));
  EXPECT_CALL(flash_ctrl_, DataErase(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(flash_ctrl_, BankErasePermsSet(kHardenedBoolFalse));

  // bootstrap_handle_erase_verify
  EXPECT_CALL(flash_ctrl_, DataEraseVerify(testing::_, kFlashCtrlEraseTypePage))
      .Times(AtLeast(1));
  EXPECT_CALL(spi_device_, FlashStatusClear());

  // bootstrap_handle_program
  ExpectSpiCmd(PageProgramCmd(FLASH_CTRL_PARAM_BYTES_PER_BANK, 1));
  ExpectSpiFlashStatusGet(true);

  EXPECT_THAT(flash_ctrl_sim_.GetFlash(), Each(Eq(FlashByte::kDefault)))
      << "Before rom_ext_bootstrap(), flash should be unmodified.";

  EXPECT_EQ(rom_ext_bootstrap(), kErrorBootstrapIllegalAddr);

  ExpectRomExtSlotA(FlashByte::kDefault);
  ExpectSuffixSlotA(FlashByte::kErasedVerified);
  ExpectRomExtSlotB(FlashByte::kDefault);
  ExpectSuffixSlotB(FlashByte::kErasedVerified);
}

}  // namespace
}  // namespace rom_ext_bootstrap_unittest
