#include <BoardConfig.h>

#if FREEINK_DEVICE_EMINIMAL

#include <It8951Config.h>

namespace freeink {

// e-Minimal 7.8": the IT8951 wiring and per-panel values the EMINIMAL_78 profile
// cannot carry. Every number here was measured on the bench at G2 (2026-09-18)
// against the Waveshare 7.8inch e-Paper HAT that ships with this device; see
// e-Minimal-firmware/STATUS.md "Measured" for the runs.
//
// The waveform mode NUMBERS are per LUT, not per controller. This panel reports
// LUT "M841", which orders them INIT 0 / DU 1 / GC16 2 / GL16 3 / GLR16 4 /
// GLD16 5 / A2 6 / DU4 7 -- M5Paper's LUT swaps the last two (DU4 6, A2 7), which
// is why the driver's default grayMode of 6 would run our anti-aliased pages
// through the 2-level A2 waveform and flatten every gray edge.
const It8951Config& eminimal78It8951Config() {
  static const It8951Config cfg = {
      17,        // miso: the HAT's own MISO. The driver default (13) is this board's
                 // SD D0 -- the card is on SDMMC, the panel has a bus of its own.
      10000000,  // spiHz: 10 MHz. 13.3 was also clean on jumpers; 16 stalled HRDY on
                 // one run in four; 20 corrupts the 20-word GET_DEV_INFO read.
                 // Re-sweep on a PCB.
      IT8951_ROTATE_AUTO,  // panel reports 1872x1404 landscape -> rotation 0
      1920,      // vcomMv: -1.92 V from the FPC's QR sticker. The controller shipped
                 // at -2.50 V, which left a checkerboard ghost through a GC16; and
                 // an SPI-set VCOM does not survive a reset, so begin() must set it
                 // on every boot -- which it does.
      2,         // fullMode  = GC16, 525 ms. The ghost-clear.
      1,         // fastMode  = DU, 299 ms, 2-level differential. A2 (6) is 192 ms
                 // but ghosts harder; judge on real pages at G4 before switching.
      7,         // grayMode  = DU4 on M841 (see above), 359 ms, 4-level direct.
      8,         // ghostClearInterval: GC16 every 8 differential refreshes.
      0x00124850,  // imgBufFallbackAddr: this HAT's buffer base per GET_DEV_INFO.
  };
  return cfg;
}

}  // namespace freeink

#endif  // FREEINK_DEVICE_EMINIMAL
