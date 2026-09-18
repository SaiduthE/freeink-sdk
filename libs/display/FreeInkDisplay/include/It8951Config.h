#pragma once

#include <Arduino.h>

namespace freeink {

// cfg.rotation sentinel: pick 0° or 90° at begin() from the panel's reported
// orientation so the landscape framebuffer lands upright on the portrait panel.
constexpr uint16_t IT8951_ROTATE_AUTO = 0xFF;

// IT8951 wiring and per-panel values. Geometry is not here; it comes from the
// active BoardProfile (and GET_DEV_INFO at runtime), like all drivers. A board
// supplies one of these via -DFREEINK_IT8951_CONFIG=yourConfig, or the SDK's
// board-support library does (BoardEMinimal78); M5Paper uses the driver default.
//
// The waveform mode numbers are a property of the panel's LUT, not the
// controller: M5Paper's LUT has DU4 at 6 and A2 at 7, Waveshare's M841 has A2 at
// 6 and DU4 at 7. Read the LUT string GET_DEV_INFO returns before copying modes.
struct It8951Config {
  int8_t miso;        // SPI MISO (IT8951 register / device-info reads)
  uint32_t spiHz;     // SPI clock
  uint16_t rotation;  // LD_IMG rotation 0/1/2/3 = 0/90/180/270°, or IT8951_ROTATE_AUTO
  uint16_t vcomMv;    // VCOM magnitude in mV (e.g. 2300 = -2.30 V); 0 = keep panel OTP
  uint8_t fullMode;   // clearing refresh (GC16 = 2). Used for Full/Half, wake from
                      // standby, and the periodic ghost-clear below.
  uint8_t fastMode;   // B/W page turns (DU = 1, 2-level differential).
  uint8_t grayMode;   // anti-aliased grayscale pages. DU4 is a 4-level DIRECT
                      // update: differential, so only changed pixels (the AA glyph
                      // edges) move — no flash. GC16 (2) would drive every pixel.
  uint16_t ghostClearInterval;  // promote a differential (DU/DU4) refresh to a GC16
                                // ghost-clear every N partials (0 = never). Keeps
                                // DU/DU4 residue from accumulating across menu and
                                // activity navigation, with no firmware involvement.
  uint32_t imgBufFallbackAddr;  // used only if GET_DEV_INFO returns no buffer address
};

}  // namespace freeink
