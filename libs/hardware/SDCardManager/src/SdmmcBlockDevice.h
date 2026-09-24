#pragma once

// FreeInk SDK — native ESP-IDF SDMMC block device for SdFat.
//
// SdFat has no ESP32 SDIO/SDMMC driver, so boards wired for 4-bit SDMMC (e.g.
// de-link) can't use its SPI card path. This adapter implements SdFat's
// FsBlockDeviceInterface on top of the ESP-IDF `sdmmc` host + `sdmmc_cmd` sector
// API, so a plain FsVolume mounts on it and hands back ordinary FsFile objects —
// the public SDCardManager API (and CrossPoint's HalFile, which stores FsFile by
// value) keeps working unchanged. Only compiled when FREEINK_SD_SDMMC is set.
//
// Hardware-validated in 1-bit mode on the Xteink X4 Pro (ESP32-S3, SSD1677 build);
// also matches the de-link board's 4-bit native-SDMMC FsFile path. The X4 Pro mount
// needs an active-LOW power-enable power-cycle per attempt (see SdmmcBlockDevice.cpp).

#include <BoardConfig.h>

#if FREEINK_SD_SDMMC

// Needs SdFat built with -DUSE_BLOCK_DEVICE_INTERFACE=1 so FsBlockDevice resolves
// to the generic FsBlockDeviceInterface (set in the de-link build env).
#include <SdFat.h>  // FsBlockDeviceInterface, Sector_t

namespace freeink {

class SdmmcBlockDevice : public FsBlockDeviceInterface {
 public:
  // Bring up the SDMMC host + slot from the board's pin map and initialise the
  // card. Returns false (and leaves the device unusable) on any failure.
  bool begin(const BoardConfig::SdmmcPins& pins);
  void end() override;

  bool isBusy() override { return false; }
  bool readSector(Sector_t sector, uint8_t* dst) override { return readSectors(sector, dst, 1); }
  bool readSectors(Sector_t sector, uint8_t* dst, size_t ns) override;
  bool writeSector(Sector_t sector, const uint8_t* src) override { return writeSectors(sector, src, 1); }
  bool writeSectors(Sector_t sector, const uint8_t* src, size_t ns) override;
  Sector_t sectorCount() override;
  bool syncDevice() override { return true; }

 private:
  static constexpr size_t kSectorSize = 512;
  static constexpr size_t kMaxTransferSectors = 8;
  // Recovery back-off: a card that keeps failing (e.g. genuinely gone, or a
  // socket fault) must not be re-mounted on every single sector op — that
  // would busy-loop the SDMMC bus. Cap it at a few attempts per rolling minute.
  static constexpr uint32_t kRecoveryWindowMs = 60000;
  static constexpr int kMaxRecoveriesPerWindow = 3;

  // esp-idf's sdmmc_card_t is a typedef of an anonymous struct, so it can't be
  // forward-declared here (a `struct sdmmc_card_t;` tag is a different, conflicting
  // type). Hold it opaquely and cast in the .cpp, where the esp-idf header is included.
  void* _card = nullptr;
  uint8_t* _dmaBuffer = nullptr;
  // Pin map captured in begin(), replayed by recover() to reinit the host/slot
  // without needing the caller to pass it again.
  BoardConfig::SdmmcPins _pins{};
  // Timestamps (millis()) of the last few recovery attempts, used as a ring
  // buffer to enforce kMaxRecoveriesPerWindow. 0 = unused slot.
  uint32_t _recoveryAttemptMs[kMaxRecoveriesPerWindow] = {0};
  int _recoveryAttemptIdx = 0;

  // Host/slot/card bring-up shared by begin() (first mount) and recover()
  // (re-mount after a wedged card): assumes _pins and _dmaBuffer are already
  // set. Power-cycles the configured sd.powerEnable gate (if any) and retries
  // init + a real sector-0 read up to 4 times, same as the OEM mountSD.
  bool mount();
  // Called from readSectors/writeSectors on a failed transfer. Deinits the
  // host and card, then re-mounts via mount(). Rate-limited to
  // kMaxRecoveriesPerWindow per kRecoveryWindowMs; logs
  // "[SD] recovered card after <err>" or "[SD] recovery failed".
  bool recover(const char* opName, int err);
};

}  // namespace freeink

#endif  // FREEINK_SD_SDMMC
