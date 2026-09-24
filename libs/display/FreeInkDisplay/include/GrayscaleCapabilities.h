#pragma once

#include <stdint.h>

namespace freeink {

// Overlay uses the existing base + LSB/MSB mask API. Absolute requires planes
// containing every pixel, including the B/W background. Direct uses that same
// encoding with one combined activation using the full-quality image waveform.
// Querying selects no mode.
enum class GrayscaleMode : uint8_t { Overlay, Absolute, Direct };
enum class GrayscaleEncoding : uint8_t { Unsupported, OverlayMasks, AbsolutePlanes };
enum class GrayscaleBase : uint8_t { Separate, Combined };

struct GrayscaleCapabilities {
  // Overlay masks: black/white=00 (distinguished by the B/W base), dark=11,
  // light=01, in (LSB, MSB) order. Absolute planes: black=00, dark=10,
  // light=01, white=11. Encoding does not promise independence from prior ink.
  GrayscaleEncoding encoding = GrayscaleEncoding::Unsupported;
  GrayscaleBase base = GrayscaleBase::Separate;
  bool stripUploads = false;
  // An ordinary asynchronous B/W refresh is a valid base for this mode.
  bool asyncBase = false;
  // Plane staging touches no SPI and is safe during a pending B/W waveform.
  bool stagingWhileBusy = false;

  constexpr bool supported() const { return encoding != GrayscaleEncoding::Unsupported; }
};

// Row source for the raw 16-level (4bpp) frame path (PanelDriver::
// displayGray4Rows and the facade's displayGray4Rows): fill(ctx, row, dst)
// writes framebuffer row `row` as one packed 4bpp row -- widthBytes * 4 bytes,
// two pixels per byte, the left pixel in the high nibble, 0x0 black .. 0xF
// white -- into dst, internal RAM that is valid for that call only. Called
// once per row, in order, on the calling task; it must not block or allocate
// (the row is built while the previous chunk is on the wire).
using Gray4RowFill = void (*)(void* ctx, uint16_t row, uint8_t* dst);

}  // namespace freeink
