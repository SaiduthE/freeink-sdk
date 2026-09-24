#include "It8951Driver.h"

#include <BoardConfig.h>

// Linked only for the IT8951 devices (M5Paper v1.1 on the classic ESP32,
// e-Minimal 7.8" on the ESP32-S3). The body is plain Arduino SPIClass, so it is
// not tied to one MCU family.
#if FREEINK_DRIVER_IT8951

#include <driver/gpio.h>

#include <cstring>

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "soc/soc_caps.h"

// DMA bulk write. Arduino's SPIClass is FIFO-polled: writeBytes() refills the
// 64-byte FIFO and spins, so the CPU cannot build the next row while one is on
// the wire, and every row's combine (base+LSB+MSB -> 4bpp, ~100 ms a grey page)
// or bit-reverse adds straight onto the ~526 ms of wire time. With this on, the
// image burst is handed to an ESP-IDF spi_master device on the OTHER SPI host,
// fed from DMA-capable internal buffers: chunk N+1 is built while chunk N is
// on the wire. Commands, register reads and HRDY-paced words stay on the
// Arduino bus; only the SCLK/MOSI pads move to the DMA host for the length of
// a burst (GPIO-matrix output select, two register writes each way), under
// the Arduino bus lock (beginTransaction) and with CS held low by this driver.
// -DIT8951_DMA_LOAD=0 compiles the polled path only; at runtime, a failed
// setup or any DMA error leaves the driver on the polled path.
#ifndef IT8951_DMA_LOAD
#define IT8951_DMA_LOAD 1
#endif
#if IT8951_DMA_LOAD && SOC_SPI_PERIPH_NUM >= 3
#define IT8951_DMA_ENABLED 1
#include <driver/spi_master.h>
#include <esp_rom_gpio.h>
#include <soc/spi_periph.h>
#else
#define IT8951_DMA_ENABLED 0
#endif

namespace freeink {
namespace {
#if IT8951_DMA_ENABLED
// The global SPIClass `SPI` (what _spi refers to) is VSPI = SPI3_HOST on the
// classic ESP32 (M5Paper) and FSPI = SPI2_HOST on the S3 (eMinimal). The DMA
// device takes the other general-purpose host. A board whose SD card has its
// own SPIClass(HSPI) -- BoardConfig sd.separateSpi -- already owns that host,
// so the DMA path stays off there (initDmaLoad checks).
#if CONFIG_IDF_TARGET_ESP32
constexpr spi_host_device_t ARDUINO_SPI_HOST = SPI3_HOST;
constexpr spi_host_device_t DMA_SPI_HOST = SPI2_HOST;
#else
constexpr spi_host_device_t ARDUINO_SPI_HOST = SPI2_HOST;
constexpr spi_host_device_t DMA_SPI_HOST = SPI3_HOST;
#endif
// Bytes per DMA chunk (whole rows; at least one full 4bpp row). 2 KB is two
// 936-byte rows on the 7.8" panel: 702 transactions a frame, each costing a
// few us of ISR hand-off, ~5 ms in all, for 3 x 2 KB of internal RAM.
constexpr uint16_t DMA_CHUNK_TARGET = 2048;
// A chunk of 2 KB takes ~0.8 ms at 20 MHz; a second without a completion
// means the peripheral is wedged.
constexpr TickType_t DMA_TIMEOUT_TICKS = pdMS_TO_TICKS(1000);
#endif

// SPI preamble words (sent MSB-first before each command/data/read).
constexpr uint16_t PRE_CMD = 0x6000;  // command write
constexpr uint16_t PRE_WR = 0x0000;   // data write
constexpr uint16_t PRE_RD = 0x1000;   // data read

// IT8951 system / I80 commands.
constexpr uint16_t CMD_SYS_RUN = 0x0001;
constexpr uint16_t CMD_STANDBY = 0x0002;
constexpr uint16_t CMD_SLEEP = 0x0003;
constexpr uint16_t CMD_REG_RD = 0x0010;
constexpr uint16_t CMD_REG_WR = 0x0011;
constexpr uint16_t CMD_LD_IMG_AREA = 0x0021;
constexpr uint16_t CMD_LD_IMG_END = 0x0022;
constexpr uint16_t CMD_DEV_INFO = 0x0302;
constexpr uint16_t CMD_DPY_AREA = 0x0034;
constexpr uint16_t CMD_VCOM = 0x0039;

// Registers.
constexpr uint16_t REG_I80CPCR = 0x0004;  // host packed-write enable
constexpr uint16_t REG_LISAR = 0x0208;    // load-image start address (low word; +2 = high word)
constexpr uint16_t REG_UP1SR = 0x1138;    // update parameter 1: +2 bit 2 = read frame memory as a 1bpp bitmap
constexpr uint16_t REG_LUTAFSR = 0x1224;  // LUT engine busy (0 = idle)
constexpr uint16_t REG_BGVR = 0x1250;     // bitmap colour table: [7:0] level for a 1 bit, [15:8] for a 0 bit

// LD_IMG arg fields: (endian << 8) | (bpp << 4) | rotation.
constexpr uint16_t BPP_2 = 0x00;       // 2 bits/pixel (four levels, expanded by the controller)
constexpr uint16_t BPP_4 = 0x02;       // 4 bits/pixel (native 16-gray)
constexpr uint16_t BPP_8 = 0x03;       // 8 bits/pixel (top nibble used) -- and the bitmap load's header
constexpr uint16_t ENDIAN_BIG = 0x01;  // big-endian pixel words

// 1bpp bitmap mode. A set framebuffer bit is white in this SDK, so a 1 bit
// paints 0xF0 and a 0 bit 0x00 -- the value Waveshare's and PaperTTY's 1bpp
// paths write, both of which also draw with 1 = white. If the glass comes up
// inverted, swap the bytes (0xF000).
constexpr uint16_t BGVR_BW = 0x00F0;
// The engine reads each byte of the bitmap LSB-first: pixel 0 of the eight is
// bit 0 (Waveshare's Paint lib packs 1bpp as `0x80 >> (7 - x%8)`, PaperTTY's
// pack_1bpp puts pixel 0 at value 1). The framebuffer is MSB-first, so every
// byte is bit-reversed on the way out. If text comes up scrambled inside
// 8-px groups, this is the assumption to flip (send the bytes as they are).
constexpr bool BITMAP_LSB_FIRST = true;
// Partial bitmap loads: PaperTTY only trusts them with the height a multiple
// of 16 (and x/w of 32; loads here are always full width). Bands are widened
// to 16-row boundaries, and the bitmap-mode refresh box to 32-px columns.
constexpr uint16_t BITMAP_ROW_ALIGN = 16;
constexpr uint16_t BITMAP_COL_ALIGN = 32;

struct BitReverseTable {
  uint8_t v[256];
  constexpr BitReverseTable() : v() {
    for (unsigned i = 0; i < 256; i++) {
      unsigned b = i;
      b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
      b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
      b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
      v[i] = static_cast<uint8_t>(b);
    }
  }
};
constexpr BitReverseTable kBitRev;

// A raw 4bpp byte (two pixels, first in the high nibble) -> those two pixels
// in the driver's base/LSB/MSB plane terms, packed [5:4] base, [3:2] LSB,
// [1:0] MSB, first pixel in the higher bit of each pair. The inverse of
// gray4() on its four levels (0xF white -> base; 0xA light -> MSB; 0x5 dark ->
// LSB; 0x0 black -> nothing), so loadImageGray() on the derived planes
// reproduces a four-level frame exactly. Other levels: 0x8..0xE count as
// light, 0x1..0x7 as dark -- what grayWithin() needs (any non-white,
// non-black pixel sets a plane bit).
struct Gray4PlaneTable {
  uint8_t v[256];
  constexpr Gray4PlaneTable() : v() {
    for (unsigned i = 0; i < 256; i++) {
      unsigned bits = 0;
      for (unsigned p = 0; p < 2; p++) {
        const unsigned n = p == 0 ? (i >> 4) : (i & 0x0F);
        const unsigned s = 1 - p;
        const unsigned white = n == 0x0F ? 1 : 0;
        const unsigned msb = (!white && n >= 0x08) ? 1 : 0;
        const unsigned lsb = (!white && n != 0 && n < 0x08) ? 1 : 0;
        bits |= (white << (4 + s)) | (lsb << (2 + s)) | (msb << s);
      }
      v[i] = static_cast<uint8_t>(bits);
    }
  }
};
constexpr Gray4PlaneTable kGray4Planes;

constexpr unsigned long READY_TIMEOUT_MS = 3000;

// One pixel's 4bpp value from CrossPoint's anti-aliasing planes (base B/W + LSB +
// MSB), matching the LilyGo ED047TC1 mapping: a set base bit is white; otherwise
// MSB/LSB pick light/dark; clear is black. IT8951 4bpp: 0x0=black .. 0xF=white.
inline uint8_t gray4(uint8_t base, uint8_t lsb, uint8_t msb, uint8_t mask) {
  if (base & mask) return 0xF;  // white
  const bool l = lsb & mask, m = msb & mask;
  if (m && l) return 0x5;  // dark
  if (m) return 0xA;       // light
  if (l) return 0x5;       // dark
  return 0x0;              // black
}

// The same pixel as a 2bpp level, 3/2/1/0 -- exactly gray4()'s four values if
// the controller expands v to v*5. The IT8951E on the eMinimal bench expands
// v<<2 (white = 0xC, grey), which is why It8951LoadDepth::Bpp2 is off there.
inline uint8_t gray2(uint8_t base, uint8_t lsb, uint8_t msb, uint8_t mask) {
  if (base & mask) return 3;  // white
  const bool l = lsb & mask, m = msb & mask;
  if (m && !l) return 2;  // light
  if (m || l) return 1;   // dark
  return 0;               // black
}
}  // namespace

const It8951Config& it8951DefaultConfig() {
  static const It8951Config cfg = {
      13,                   // miso (M5Paper: GPIO13, shared with SD)
      10000000,             // 10 MHz SPI
      IT8951_ROTATE_AUTO,   // pick 0°/90° from the reported panel orientation
      0,                    // vcomMv: 0 -> keep the panel's factory OTP VCOM
      2,                    // fullMode  = GC16 (ghost-clearing; Full/Half/wake/periodic)
      1,                    // fastMode  = DU   (2-level B/W, fast)
      6,                    // grayMode  = DU4  (4-level DIRECT update: differential,
                            //                   only changed pixels move -> no flash)
      8,                    // ghostClearInterval: GC16 clear every 8 differential refreshes
      0x001236E0,           // imgBufFallbackAddr (typical M5Paper IT8951 buffer base)
  };
  return cfg;
}

It8951Driver::It8951Driver(const It8951Config& cfg)
    : _cfg(cfg),
      _spi(SPI),  // share the Arduino global bus with the SD card (see header)
      _fbW(BoardConfig::ACTIVE.displayWidth),
      _fbH(BoardConfig::ACTIVE.displayHeight),
      _fbWb(BoardConfig::ACTIVE.displayWidth / 8) {}

PanelGeometry It8951Driver::geometry() const {
  return {_fbW, _fbH, _fbWb, static_cast<uint32_t>(_fbWb) * _fbH};
}

void It8951Driver::waitReady() {
  if (_busy < 0) return;
  const unsigned long start = millis();
  while (digitalRead(_busy) == LOW) {
    if (millis() - start > READY_TIMEOUT_MS) {  // fail open rather than hang
#ifdef IT8951_PROBE_DEBUG
      if (Serial) Serial.printf("[it8951] waitReady TIMEOUT (busy pin %d stuck LOW)\n", _busy);
#endif
      return;
    }
  }
}

void It8951Driver::writeWord(uint16_t preamble, uint16_t value) {
  waitReady();
  _spi.beginTransaction(SPISettings(_cfg.spiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  _spi.transfer16(preamble);
  _spi.transfer16(value);
  digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void It8951Driver::writeCommand(uint16_t cmd) { writeWord(PRE_CMD, cmd); }
void It8951Driver::writeData(uint16_t data) { writeWord(PRE_WR, data); }

uint16_t It8951Driver::readData() {
  waitReady();
  _spi.beginTransaction(SPISettings(_cfg.spiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  _spi.transfer16(PRE_RD);
  _spi.transfer16(0x0000);  // dummy word the controller requires before valid data
  const uint16_t v = _spi.transfer16(0x0000);
  digitalWrite(_cs, HIGH);
  _spi.endTransaction();
  return v;
}

void It8951Driver::readWords(uint16_t* buf, uint16_t count) {
  waitReady();
  _spi.beginTransaction(SPISettings(_cfg.spiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  _spi.transfer16(PRE_RD);
  _spi.transfer16(0x0000);  // dummy
  for (uint16_t i = 0; i < count; i++) buf[i] = _spi.transfer16(0x0000);
  digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

void It8951Driver::writeReg(uint16_t reg, uint16_t value) {
  writeCommand(CMD_REG_WR);
  writeData(reg);
  writeData(value);
}

uint16_t It8951Driver::readReg(uint16_t reg) {
  writeCommand(CMD_REG_RD);
  writeData(reg);
  return readData();
}

void It8951Driver::systemRun() { writeCommand(CMD_SYS_RUN); }

void It8951Driver::getDeviceInfo() {
  writeCommand(CMD_DEV_INFO);
  uint16_t info[20];  // PanelW, PanelH, bufAddrL, bufAddrH, FW[8], LUT[8]
  readWords(info, 20);
  _panelW = info[0];
  _panelH = info[1];
  _imgBufAddr = (static_cast<uint32_t>(info[3]) << 16) | info[2];
  // Reject an implausible read (no MISO / cold controller) and fall back so the
  // load path still targets a valid buffer.
  if (_panelW == 0 || _panelW > 2048 || _panelH == 0 || _panelH > 2048) {
    _panelW = _fbW;
    _panelH = _fbH;
  }
  if (_imgBufAddr == 0) _imgBufAddr = _cfg.imgBufFallbackAddr;
#ifdef IT8951_PROBE_DEBUG
  // Diagnostic: raw GET_DEV_INFO read-back. Plausible values (e.g. 960x540, a
  // non-zero buffer addr, ASCII firmware string) mean SPI reads work; all-zero or
  // 0xFFFF means no MISO / cold controller / wrong pins.
  if (Serial)
    Serial.printf("[it8951] GET_DEV_INFO: panelW=%u panelH=%u imgBufAddr=0x%08lX (info[0..3]=%04X %04X %04X %04X)\n",
                  _panelW, _panelH, (unsigned long)_imgBufAddr, info[0], info[1], info[2], info[3]);
#endif
}

void It8951Driver::setTargetMemoryAddr(uint32_t addr) {
  writeReg(REG_LISAR + 2, static_cast<uint16_t>(addr >> 16));
  writeReg(REG_LISAR, static_cast<uint16_t>(addr & 0xFFFF));
}

void It8951Driver::setVcom(uint16_t mv) {
  if (mv == 0) return;  // keep factory OTP VCOM
  writeCommand(CMD_VCOM);
  writeData(0x0001);  // 1 = set
  writeData(mv);
}

void It8951Driver::setBitmapMode(bool on, bool force) {
  if (!force && on == _bitmapModeOn) return;
  waitDisplayReady();  // never change how the engine reads memory under a running refresh
  const uint16_t up1sr2 = readReg(REG_UP1SR + 2);
  writeReg(REG_UP1SR + 2, on ? static_cast<uint16_t>(up1sr2 | (1u << 2)) : static_cast<uint16_t>(up1sr2 & ~(1u << 2)));
  if (on) writeReg(REG_BGVR, BGVR_BW);
  _bitmapModeOn = on;
#ifdef IT8951_PROBE_DEBUG
  if (Serial) Serial.printf("[it8951] bitmap mode %s (UP1SR+2 was %04X)\n", on ? "ON" : "off", up1sr2);
#endif
}

void It8951Driver::displayArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode) {
  // The engine must read the memory the way the last load wrote it. INIT
  // (mode 0) ignores the memory, but keeping the bit honest costs nothing.
  setBitmapMode(_memBitmap);
  if (_memBitmap && _rotation == 0) {
    // Widen the box to 32-px columns: a bitmap-mode refresh of a box starting
    // mid-word is untested (see BITMAP_COL_ALIGN); a few extra pixels of an
    // unchanged margin are invisible.
    const uint16_t x0 = static_cast<uint16_t>(x & ~(BITMAP_COL_ALIGN - 1));
    uint32_t x1 = (static_cast<uint32_t>(x) + w + BITMAP_COL_ALIGN - 1) & ~static_cast<uint32_t>(BITMAP_COL_ALIGN - 1);
    if (x1 > _panelW) x1 = _panelW;
    x = x0;
    w = static_cast<uint16_t>(x1 - x0);
  }
  writeCommand(CMD_DPY_AREA);
  writeData(x);
  writeData(y);
  writeData(w);
  writeData(h);
  writeData(mode);
}

void It8951Driver::waitDisplayReady() {
  const unsigned long start = millis();
  while (readReg(REG_LUTAFSR) != 0) {
    if (millis() - start > READY_TIMEOUT_MS) {
#ifdef IT8951_PROBE_DEBUG
      if (Serial) Serial.printf("[it8951] waitDisplayReady TIMEOUT (LUTAFSR never cleared)\n");
#endif
      return;
    }
  }
}

// --- bulk image write -------------------------------------------------------
// HRDY is honoured where the driver has always honoured it: before the
// PRE_WR preamble that opens the burst (and before every command/data word
// around it). The rows themselves go out back to back in the one CS-low
// frame with no HRDY check, polled or DMA -- the IT8951's load FIFO keeps up
// at 20 MHz (bench, G2: 2.26 MB/s loaded = the no-load simulation, so HRDY
// flow control during the burst costs nothing).
template <typename Fill>
void It8951Driver::streamImageRows(uint16_t numRows, uint16_t rowBytes, Fill&& fill) {
  waitReady();
  _spi.beginTransaction(SPISettings(loadClockHz(), MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  _spi.transfer16(PRE_WR);  // blocking: done on the wire before the pads move
  uint16_t row = 0;         // first row not yet handed to the wire
#if IT8951_DMA_ENABLED
  if (_dmaDev && !_dmaFailed && rowBytes != 0 && rowBytes <= _dmaChunkBytes) {
    const uint16_t perChunk = static_cast<uint16_t>(_dmaChunkBytes / rowBytes);
    routeLoadPinsToDma(true);
    uint8_t inFlight = 0;
    uint8_t slot = 0;
    bool ok = true;
    uint8_t* unsent = nullptr;  // a chunk built but never queued (queue failure)
    uint16_t unsentRows = 0;
    while (row < numRows) {
      // All buffers queued: wait for the oldest to leave the wire. The chunks
      // behind it keep the bus busy while this one is refilled.
      if (inFlight == DMA_BUFS) {
        if (!retireDmaChunk()) {
          ok = false;
          break;
        }
        inFlight--;
      }
      const uint16_t left = static_cast<uint16_t>(numRows - row);
      const uint16_t n = left < perChunk ? left : perChunk;
      uint8_t* buf = _dmaBuf[slot];
      for (uint16_t k = 0; k < n; k++) fill(static_cast<uint16_t>(row + k), buf + static_cast<uint32_t>(k) * rowBytes);
      spi_transaction_t* t = &_dmaTrans[slot];
      memset(t, 0, sizeof(*t));
      t->length = static_cast<size_t>(n) * rowBytes * 8;  // bits
      t->tx_buffer = buf;
      if (spi_device_queue_trans(_dmaDev, t, DMA_TIMEOUT_TICKS) != ESP_OK) {
        unsent = buf;
        unsentRows = n;
        ok = false;
        break;
      }
      inFlight++;
      row = static_cast<uint16_t>(row + n);
      slot = static_cast<uint8_t>((slot + 1) % DMA_BUFS);
    }
    // Drain: the burst is only over once the last chunk is off the wire.
    while (inFlight > 0) {
      if (!retireDmaChunk()) {
        ok = false;
        break;
      }
      inFlight--;
    }
    routeLoadPinsToDma(false);
    if (!ok) {
      // Only a wedged peripheral gets here (a queue slot is always free and a
      // chunk takes < 1 ms). Finish this burst polled from the first unqueued
      // row -- the frame may be off by a chunk if one was cut mid-flight; the
      // next load is clean -- and stay polled from now on. A chunk built but
      // never queued goes out as it is: fill() has run for its rows already,
      // and is asked for every row exactly once.
      _dmaFailed = true;
      if (Serial) Serial.printf("[it8951] DMA bulk write failed at row %u of %u; polled SPI from now on\n", row, numRows);
      if (unsent) {
        _spi.writeBytes(unsent, static_cast<uint32_t>(unsentRows) * rowBytes);
        row = static_cast<uint16_t>(row + unsentRows);
      }
    }
  }
#endif
  for (; row < numRows; row++) {
    fill(row, _rowBuf);
    _spi.writeBytes(_rowBuf, rowBytes);
  }
  digitalWrite(_cs, HIGH);
  _spi.endTransaction();
}

bool It8951Driver::retireDmaChunk() {
#if IT8951_DMA_ENABLED
  spi_transaction_t* done = nullptr;
  return spi_device_get_trans_result(_dmaDev, &done, DMA_TIMEOUT_TICKS) == ESP_OK;
#else
  return false;
#endif
}

// The SCLK/MOSI pads are GPIO-matrix outputs; each selects one peripheral
// signal. Point them at the DMA host for a burst and back at the Arduino
// bus's signals (what spiAttachSCK/MOSI programmed) after it. Both hosts idle
// with SCLK low in mode 0 and nothing is clocked across the switch; MISO is
// input-only and stays on the Arduino bus throughout.
void It8951Driver::routeLoadPinsToDma(bool toDma) {
#if IT8951_DMA_ENABLED
  const spi_host_device_t host = toDma ? DMA_SPI_HOST : ARDUINO_SPI_HOST;
  esp_rom_gpio_connect_out_signal(static_cast<uint32_t>(_sclk), spi_periph_signal[host].spiclk_out, false, false);
  esp_rom_gpio_connect_out_signal(static_cast<uint32_t>(_mosi), spi_periph_signal[host].spid_out, false, false);
#else
  (void)toDma;
#endif
}

// Bring up the DMA device once (begin()). Any failure leaves _dmaDev null and
// every load on the polled path.
void It8951Driver::initDmaLoad() {
#if IT8951_DMA_ENABLED
  if (_dmaDev || _dmaFailed) return;
  if (_sclk < 0 || _mosi < 0 || _cs < 0) return;
  if (BoardConfig::ACTIVE.sd.separateSpi) return;  // the SD card owns the other host (see DMA_SPI_HOST)
  const uint32_t fullRow = static_cast<uint32_t>(_fbWb) * 4;  // widest row any load sends
  uint32_t chunk = fullRow > DMA_CHUNK_TARGET ? fullRow : DMA_CHUNK_TARGET;
  chunk = (chunk + 3) & ~3u;
  if (chunk > 0xFFFF) return;
  auto release = [this]() {
    for (auto& b : _dmaBuf) {
      if (b) heap_caps_free(b);
      b = nullptr;
    }
    if (_dmaTrans) heap_caps_free(_dmaTrans);
    _dmaTrans = nullptr;
  };
  // DMA reads these: internal, DMA-capable RAM (never PSRAM). Word-aligned so
  // spi_master sends them in place instead of bouncing through a copy.
  for (auto& b : _dmaBuf) {
    b = static_cast<uint8_t*>(heap_caps_aligned_alloc(4, chunk, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!b) {
      release();
      return;
    }
  }
  _dmaTrans = static_cast<spi_transaction_t*>(
      heap_caps_calloc(DMA_BUFS, sizeof(spi_transaction_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!_dmaTrans) {
    release();
    return;
  }

  spi_bus_config_t bus = {};
  bus.mosi_io_num = _mosi;
  bus.miso_io_num = -1;  // write-only; MISO stays on the Arduino bus
  bus.sclk_io_num = _sclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.data4_io_num = -1;
  bus.data5_io_num = -1;
  bus.data6_io_num = -1;
  bus.data7_io_num = -1;
  bus.max_transfer_sz = static_cast<int>(chunk);
  // GPIO matrix, never the IOMUX: the pads must stay switchable between hosts.
  bus.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS;
  esp_err_t err = spi_bus_initialize(DMA_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    routeLoadPinsToDma(false);  // whatever the failed init touched, give the pads back
    release();
    if (Serial) Serial.printf("[it8951] DMA bulk write unavailable (bus init %d); polled SPI\n", static_cast<int>(err));
    return;
  }
  spi_device_interface_config_t dev = {};
  dev.mode = 0;
  dev.clock_speed_hz = static_cast<int>(loadClockHz());
  dev.spics_io_num = -1;  // CS is this driver's GPIO, low for the whole burst
  dev.queue_size = DMA_BUFS;
  dev.flags = SPI_DEVICE_HALFDUPLEX;  // TX only
  spi_device_handle_t handle = nullptr;
  err = spi_bus_add_device(DMA_SPI_HOST, &dev, &handle);
  // spi_bus_initialize routed the pads to the DMA host: hand them back to the
  // Arduino bus until the first burst.
  routeLoadPinsToDma(false);
  if (err != ESP_OK) {
    spi_bus_free(DMA_SPI_HOST);
    release();
    if (Serial) Serial.printf("[it8951] DMA bulk write unavailable (add device %d); polled SPI\n", static_cast<int>(err));
    return;
  }
  _dmaDev = handle;
  _dmaChunkBytes = static_cast<uint16_t>(chunk);
#ifdef IT8951_TIMING_LOG
  // The clock the host really set: requests round to an 80/N rung, so this is
  // what a loadSpiHz change actually bought.
  int actualKhz = 0;
  spi_device_get_actual_freq(handle, &actualKhz);
  if (Serial)
    Serial.printf("[it8951] bulk write via DMA: SPI host %d, %u x %lu B chunks, %lu Hz asked, %d kHz actual\n",
                  static_cast<int>(DMA_SPI_HOST), static_cast<unsigned>(DMA_BUFS), static_cast<unsigned long>(chunk),
                  static_cast<unsigned long>(loadClockHz()), actualKhz);
#endif
#endif
}

// Expand the 1bpp landscape framebuffer to the IT8951's 4bpp packing and stream it
// into controller SRAM. White bit (1) -> 0xF, black bit (0) -> 0x0; two pixels per
// byte, leftmost pixel in the high nibble. The whole image rides one CS-low burst
// after a single PRE_WR preamble (per-word framing would be far too slow).
void It8951Driver::loadImageFull(const uint8_t* fb) {
  loadImageArea(fb, 0, static_cast<uint16_t>(_fbWb - 1), 0, static_cast<uint16_t>(_fbH - 1));
}

// The rectangle case. LD_IMG_AREA takes x/w in pixels; a framebuffer byte is 8
// pixels, which keeps every edge on the 4-px word boundary the 4bpp packing
// wants. The controller places the rows at (x, y) in its frame memory, so the
// pixels outside the rectangle keep whatever the last load put there.
void It8951Driver::loadImageArea(const uint8_t* fb, uint16_t xb0, uint16_t xb1, uint16_t y0, uint16_t y1) {
  if (!_rowBuf) return;  // begin() could not allocate the row buffer; nothing to stream
  if (bitmapLoads() && xb0 == 0 && xb1 == _fbWb - 1) {
    loadImageBitmap(fb, y0, y1);
    return;
  }
  if (!_running) {
    systemRun();
    _running = true;
  }
#ifdef IT8951_TIMING_LOG
  const uint32_t tLoad = micros();
#endif
  setTargetMemoryAddr(_imgBufAddr);

  // Levels over a bitmap: the two formats cannot share the frame memory, so a
  // partial load in this format replaces the whole frame (and with it any gray).
  if (_memBitmap) {
    xb0 = 0;
    xb1 = static_cast<uint16_t>(_fbWb - 1);
    y0 = 0;
    y1 = static_cast<uint16_t>(_fbH - 1);
    _memHasGray = false;
  }
  const bool twoBpp = _cfg.loadDepth == It8951LoadDepth::Bpp2;
  const uint16_t widthBytes = static_cast<uint16_t>(xb1 - xb0 + 1);
  const uint16_t arg =
      static_cast<uint16_t>((ENDIAN_BIG << 8) | ((twoBpp ? BPP_2 : BPP_4) << 4) | (_rotation & 0x03));
  writeCommand(CMD_LD_IMG_AREA);
  writeData(arg);
  writeData(static_cast<uint16_t>(xb0 * 8));         // x (image space)
  writeData(y0);                                     // y
  writeData(static_cast<uint16_t>(widthBytes * 8));  // w
  writeData(static_cast<uint16_t>(y1 - y0 + 1));     // h

  // 4bpp: 2 px per byte, a framebuffer byte becomes 4; 2bpp: 4 px per byte, 2.
  const uint16_t rowOutBytes = static_cast<uint16_t>(widthBytes * (twoBpp ? 2 : 4));

  streamImageRows(static_cast<uint16_t>(y1 - y0 + 1), rowOutBytes, [&](uint16_t i, uint8_t* rowBuf) {
    const uint8_t* src = fb + static_cast<uint32_t>(y0 + i) * _fbWb + xb0;
    uint16_t o = 0;
    if (twoBpp) {
      // Spread each 1-bit pixel to 2 bits (1 -> 11 white, 0 -> 00 black),
      // leftmost pixel in the top bits of the first byte.
      for (uint16_t xb = 0; xb < widthBytes; xb++) {
        const uint8_t b = src[xb];
        rowBuf[o++] = static_cast<uint8_t>(((b & 0x80) ? 0xC0 : 0) | ((b & 0x40) ? 0x30 : 0) |
                                           ((b & 0x20) ? 0x0C : 0) | ((b & 0x10) ? 0x03 : 0));
        rowBuf[o++] = static_cast<uint8_t>(((b & 0x08) ? 0xC0 : 0) | ((b & 0x04) ? 0x30 : 0) |
                                           ((b & 0x02) ? 0x0C : 0) | ((b & 0x01) ? 0x03 : 0));
      }
    } else {
      for (uint16_t xb = 0; xb < widthBytes; xb++) {
        const uint8_t b = src[xb];
        rowBuf[o++] = static_cast<uint8_t>(((b & 0x80) ? 0xF0 : 0x00) | ((b & 0x40) ? 0x0F : 0x00));
        rowBuf[o++] = static_cast<uint8_t>(((b & 0x20) ? 0xF0 : 0x00) | ((b & 0x10) ? 0x0F : 0x00));
        rowBuf[o++] = static_cast<uint8_t>(((b & 0x08) ? 0xF0 : 0x00) | ((b & 0x04) ? 0x0F : 0x00));
        rowBuf[o++] = static_cast<uint8_t>(((b & 0x02) ? 0xF0 : 0x00) | ((b & 0x01) ? 0x0F : 0x00));
      }
    }
  });

  writeCommand(CMD_LD_IMG_END);
  _memBitmap = false;
#ifdef IT8951_TIMING_LOG
  _loadUs += micros() - tLoad;
  _loadBytes += static_cast<uint32_t>(widthBytes) * (twoBpp ? 2 : 4) * (y1 - y0 + 1);
#endif
}

// The bitmap load is the trick Waveshare's EPD_IT8951_1bp_Refresh and
// PaperTTY's driver share: the packed 1-bit rows go up under an 8bpp header
// declaring the image a panel-width/8 "pixels" wide, so the load engine
// stores the bytes untouched at the start of each row of the (8-bit-pitch)
// frame memory, and the display engine with UP1SR's bitmap bit set reads
// them back as eight pixels each, painting the two levels from BGVR. No
// expansion, so black and white are exact -- the thing 2bpp could not do --
// and the whole frame is 328 KB instead of 1.31 MB.
//
// Rotation 0 only (bitmapLoads() checks): the rotator would rotate the byte
// image. Rows are widened to 16-row boundaries (see BITMAP_ROW_ALIGN); the
// extra rows carry the same content the memory already holds.
void It8951Driver::loadImageBitmap(const uint8_t* fb, uint16_t y0, uint16_t y1) {
  if (!_rowBuf || !fb) return;
  if (!_running) {
    systemRun();
    _running = true;
  }
#ifdef IT8951_TIMING_LOG
  const uint32_t tLoad = micros();
#endif
  setTargetMemoryAddr(_imgBufAddr);

  if (!_memBitmap) {
    // A bitmap over levels: replace the whole frame (see loadImageArea).
    y0 = 0;
    y1 = static_cast<uint16_t>(_fbH - 1);
    _memHasGray = false;
  } else {
    y0 = static_cast<uint16_t>(y0 & ~(BITMAP_ROW_ALIGN - 1));
    const uint32_t end = (static_cast<uint32_t>(y1) + BITMAP_ROW_ALIGN) & ~static_cast<uint32_t>(BITMAP_ROW_ALIGN - 1);
    y1 = static_cast<uint16_t>((end > _fbH ? _fbH : end) - 1);
  }

  const uint16_t arg = static_cast<uint16_t>((ENDIAN_BIG << 8) | (BPP_8 << 4) | 0);
  writeCommand(CMD_LD_IMG_AREA);
  writeData(arg);
  writeData(0);                                  // x, in bytes of the bitmap row
  writeData(y0);                                 // y
  writeData(_fbWb);                              // w, in bytes (234 on 1872 px)
  writeData(static_cast<uint16_t>(y1 - y0 + 1));  // h

#ifdef IT8951_PROBE_DEBUG
  if (Serial) Serial.printf("[it8951] bitmap load rows %u..%u (%u B/row)\n", y0, y1, _fbWb);
#endif

  // Rows need _fbWb bytes (the row buffers hold _fbWb * 4). The per-row bit
  // reverse is the transform the DMA path hides behind the wire.
  streamImageRows(static_cast<uint16_t>(y1 - y0 + 1), _fbWb, [&](uint16_t i, uint8_t* rowBuf) {
    const uint8_t* src = fb + static_cast<uint32_t>(y0 + i) * _fbWb;
    if (BITMAP_LSB_FIRST) {
      for (uint16_t xb = 0; xb < _fbWb; xb++) rowBuf[xb] = kBitRev.v[src[xb]];
    } else {
      memcpy(rowBuf, src, _fbWb);  // PSRAM -> internal RAM for the SPI FIFO / DMA
    }
  });

  writeCommand(CMD_LD_IMG_END);
  _memBitmap = true;
#ifdef IT8951_TIMING_LOG
  _loadUs += micros() - tLoad;
  _loadBytes += static_cast<uint32_t>(_fbWb) * (y1 - y0 + 1);
#endif
}

// Row-by-row memcmp against the snapshot, then the first and last differing byte
// of each differing row. ~330 KB read from each buffer, a few tens of ms from
// PSRAM -- small next to the 1.1 s push it saves when the box is a menu band.
bool It8951Driver::diffBox(const uint8_t* fb, uint16_t& xb0, uint16_t& xb1, uint16_t& y0, uint16_t& y1,
                           uint32_t& changedBytes) const {
  bool any = false;
  changedBytes = 0;
  for (uint16_t y = 0; y < _fbH; y++) {
    const uint8_t* a = fb + static_cast<uint32_t>(y) * _fbWb;
    const uint8_t* b = _base + static_cast<uint32_t>(y) * _fbWb;
    if (memcmp(a, b, _fbWb) == 0) continue;
    uint16_t lo = 0;
    while (a[lo] == b[lo]) lo++;
    uint16_t hi = static_cast<uint16_t>(_fbWb - 1);
    while (a[hi] == b[hi]) hi--;
    changedBytes += static_cast<uint32_t>(hi - lo + 1);
    if (!any) {
      any = true;
      y0 = y;
      xb0 = lo;
      xb1 = hi;
    } else {
      if (lo < xb0) xb0 = lo;
      if (hi > xb1) xb1 = hi;
    }
    y1 = y;
  }
  return any;
}

void It8951Driver::buildGrayTable() {
  if (_grayTab) return;
  _grayTab = static_cast<uint16_t*>(heap_caps_malloc(4096 * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!_grayTab) return;  // loadImageGray falls back to gray4() per pixel
  for (uint16_t b = 0; b < 16; b++) {
    for (uint16_t l = 0; l < 16; l++) {
      for (uint16_t m = 0; m < 16; m++) {
        const uint8_t bb = static_cast<uint8_t>(b), lb = static_cast<uint8_t>(l), mb = static_cast<uint8_t>(m);
        const uint8_t hi = static_cast<uint8_t>((gray4(bb, lb, mb, 0x8) << 4) | gray4(bb, lb, mb, 0x4));
        const uint8_t lo = static_cast<uint8_t>((gray4(bb, lb, mb, 0x2) << 4) | gray4(bb, lb, mb, 0x1));
        _grayTab[(b << 8) | (l << 4) | m] = static_cast<uint16_t>((hi << 8) | lo);
      }
    }
  }
}

// Same rotation/streaming path as loadImageFull, but each pixel's nibble is the
// 4-level gray reconstructed from the B/W base + the buffered LSB/MSB planes.
void It8951Driver::loadImageGray(const uint8_t* base) {
  if (!_rowBuf) return;
  if (!_running) {
    systemRun();
    _running = true;
  }
#ifdef IT8951_TIMING_LOG
  const uint32_t tLoad = micros();
#endif
  setTargetMemoryAddr(_imgBufAddr);

  const bool twoBpp = _cfg.loadDepth == It8951LoadDepth::Bpp2;
  const uint16_t arg =
      static_cast<uint16_t>((ENDIAN_BIG << 8) | ((twoBpp ? BPP_2 : BPP_4) << 4) | (_rotation & 0x03));
  writeCommand(CMD_LD_IMG_AREA);
  writeData(arg);
  writeData(0);     // x
  writeData(0);     // y
  writeData(_fbW);  // w (image space)
  writeData(_fbH);  // h

  const bool haveGray = _gLsb && _gMsb;
  const uint16_t maxXb = _fbWb;
  const uint16_t rowOutBytes = static_cast<uint16_t>(maxXb * (twoBpp ? 2 : 4));

  // The combine (~70 us a row from PSRAM) is what the DMA path overlaps with
  // the previous chunk's ~375 us on the wire.
  streamImageRows(_fbH, rowOutBytes, [&](uint16_t y, uint8_t* rowBuf) {
    const uint32_t rowOff = static_cast<uint32_t>(y) * _fbWb;
    const uint8_t* brow = base + rowOff;
    const uint8_t* lrow = haveGray ? _gLsb + rowOff : nullptr;
    const uint8_t* mrow = haveGray ? _gMsb + rowOff : nullptr;
    uint16_t o = 0;
    if (twoBpp) {
      for (uint16_t xb = 0; xb < maxXb; xb++) {
        const uint8_t bb = brow[xb];
        const uint8_t lb = haveGray ? lrow[xb] : 0;
        const uint8_t mb = haveGray ? mrow[xb] : 0;
        rowBuf[o++] = static_cast<uint8_t>((gray2(bb, lb, mb, 0x80) << 6) | (gray2(bb, lb, mb, 0x40) << 4) |
                                           (gray2(bb, lb, mb, 0x20) << 2) | gray2(bb, lb, mb, 0x10));
        rowBuf[o++] = static_cast<uint8_t>((gray2(bb, lb, mb, 0x08) << 6) | (gray2(bb, lb, mb, 0x04) << 4) |
                                           (gray2(bb, lb, mb, 0x02) << 2) | gray2(bb, lb, mb, 0x01));
      }
    } else if (_grayTab && haveGray) {
      // Two table lookups per framebuffer byte: the high nibbles of the three
      // planes pick the first two output bytes, the low nibbles the next two.
      for (uint16_t xb = 0; xb < maxXb; xb++) {
        const uint8_t bb = brow[xb], lb = lrow[xb], mb = mrow[xb];
        const uint16_t hi = _grayTab[((bb & 0xF0) << 4) | (lb & 0xF0) | (mb >> 4)];
        const uint16_t lo = _grayTab[((bb & 0x0F) << 8) | ((lb & 0x0F) << 4) | (mb & 0x0F)];
        rowBuf[o++] = static_cast<uint8_t>(hi >> 8);
        rowBuf[o++] = static_cast<uint8_t>(hi);
        rowBuf[o++] = static_cast<uint8_t>(lo >> 8);
        rowBuf[o++] = static_cast<uint8_t>(lo);
      }
    } else {
      for (uint16_t xb = 0; xb < maxXb; xb++) {
        const uint8_t bb = brow[xb];
        const uint8_t lb = haveGray ? lrow[xb] : 0;
        const uint8_t mb = haveGray ? mrow[xb] : 0;
        rowBuf[o++] = static_cast<uint8_t>((gray4(bb, lb, mb, 0x80) << 4) | gray4(bb, lb, mb, 0x40));
        rowBuf[o++] = static_cast<uint8_t>((gray4(bb, lb, mb, 0x20) << 4) | gray4(bb, lb, mb, 0x10));
        rowBuf[o++] = static_cast<uint8_t>((gray4(bb, lb, mb, 0x08) << 4) | gray4(bb, lb, mb, 0x04));
        rowBuf[o++] = static_cast<uint8_t>((gray4(bb, lb, mb, 0x02) << 4) | gray4(bb, lb, mb, 0x01));
      }
    }
  });

  writeCommand(CMD_LD_IMG_END);
  _memBitmap = false;  // whole frame, levels format
#ifdef IT8951_TIMING_LOG
  _loadUs += micros() - tLoad;
  _loadBytes += static_cast<uint32_t>(_fbWb) * 4 * _fbH;
#endif
}

// A host-built 4bpp frame: the same LD_IMG_AREA header, rotation and byte
// stream as loadImageGray() (whole frame, image space _fbW x _fbH, 4bpp,
// ENDIAN_BIG), but the bytes come ready-made: `fill` builds each row straight
// in the DMA/row buffer (the facade's displayGray4: a memcpy out of the host's
// PSRAM frame; displayGray4Rows: whatever the host composes). While the row is
// in internal RAM its pixels are folded back into _base/_gLsb/_gMsb (see
// Gray4PlaneTable) so the next display() can diff against the frame now in
// controller memory, and the base row is copied out to `baseOut` when the host
// asked for it. Without planes of our own (their PSRAM allocation failed) the
// base row is still derived, into baseOut alone: the host's copy never depends
// on ours. Always 4bpp on the wire, whatever _cfg.loadDepth says: the input is
// 4bpp.
bool It8951Driver::loadImageGray4(Gray4RowFill fill, void* ctx, uint8_t* baseOut) {
  if (!_rowBuf || !fill) return false;
  if (!_running) {
    systemRun();
    _running = true;
  }
#ifdef IT8951_TIMING_LOG
  const uint32_t tLoad = micros();
#endif
  setTargetMemoryAddr(_imgBufAddr);

  const uint16_t arg = static_cast<uint16_t>((ENDIAN_BIG << 8) | (BPP_4 << 4) | (_rotation & 0x03));
  writeCommand(CMD_LD_IMG_AREA);
  writeData(arg);
  writeData(0);     // x
  writeData(0);     // y
  writeData(_fbW);  // w (image space)
  writeData(_fbH);  // h

  const uint16_t rowBytes = static_cast<uint16_t>(_fbWb * 4);  // _fbW / 2
  const bool derive = _base && _gLsb && _gMsb;
  streamImageRows(_fbH, rowBytes, [&](uint16_t y, uint8_t* rowBuf) {
    fill(ctx, y, rowBuf);
    if (!derive && !baseOut) return;
    const uint32_t off = static_cast<uint32_t>(y) * _fbWb;
    const uint8_t* s = rowBuf;
    // Four 2-pixel lookups -> one framebuffer byte (8 px, first pixel MSB) per plane.
    auto pack = [](unsigned t0, unsigned t1, unsigned t2, unsigned t3, unsigned sh) {
      return static_cast<uint8_t>((((t0 >> sh) & 3u) << 6) | (((t1 >> sh) & 3u) << 4) | (((t2 >> sh) & 3u) << 2) |
                                  ((t3 >> sh) & 3u));
    };
    if (!derive) {
      // No planes of our own: the host's base only.
      uint8_t* b = baseOut + off;
      for (uint16_t xb = 0; xb < _fbWb; xb++, s += 4) {
        b[xb] = pack(kGray4Planes.v[s[0]], kGray4Planes.v[s[1]], kGray4Planes.v[s[2]], kGray4Planes.v[s[3]], 4);
      }
      return;
    }
    uint8_t* b = _base + off;
    uint8_t* l = _gLsb + off;
    uint8_t* m = _gMsb + off;
    for (uint16_t xb = 0; xb < _fbWb; xb++, s += 4) {
      const unsigned t0 = kGray4Planes.v[s[0]], t1 = kGray4Planes.v[s[1]];
      const unsigned t2 = kGray4Planes.v[s[2]], t3 = kGray4Planes.v[s[3]];
      b[xb] = pack(t0, t1, t2, t3, 4);
      l[xb] = pack(t0, t1, t2, t3, 2);
      m[xb] = pack(t0, t1, t2, t3, 0);
    }
    if (baseOut) memcpy(baseOut + off, b, _fbWb);
  });

  writeCommand(CMD_LD_IMG_END);
  _memBitmap = false;  // whole frame, levels format
#ifdef IT8951_TIMING_LOG
  _loadUs += micros() - tLoad;
  _loadBytes += static_cast<uint32_t>(rowBytes) * _fbH;
#endif
  return derive;
}

void It8951Driver::begin(EpdBus& bus) {
  (void)bus;  // external bus: this driver owns SPI
  const auto& d = BoardConfig::ACTIVE.display;
  _sclk = d.sclk;
  _mosi = d.mosi;
  _cs = d.cs;
  _busy = d.busy;
  _pwrEn = d.powerEnable;
  _miso = _cfg.miso;
  _sdCs = BoardConfig::ACTIVE.sd.cs;  // shared SPI bus: hold SD de-selected

  if (_cs >= 0) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
  }
  if (_sdCs >= 0) {
    pinMode(_sdCs, OUTPUT);
    digitalWrite(_sdCs, HIGH);
  }
  if (_busy >= 0) pinMode(_busy, INPUT);
  if (_pwrEn >= 0) {
    // PowerManager::powerDownRailsForSleep() latches this rail LOW through deep
    // sleep (the hold survives wake) and a held pad silently ignores writes.
    // Release it first — this driver bypasses EpdBus, which does the same
    // release for the other panels.
    gpio_hold_dis(static_cast<gpio_num_t>(_pwrEn));
    pinMode(_pwrEn, OUTPUT);
    digitalWrite(_pwrEn, HIGH);  // enable the EPD power rail
    delay(100);
  }

  // Bring up the shared global SPI bus. SDCardManager::begin() may have already
  // begun this same object with the SD pins; that's fine *only* because a
  // shared-bus board wires the display and SD on the same SCLK/MOSI/MISO, so both
  // begins program identical pins (whichever runs last is a no-op re-init). The
  // contract: on a shared bus (display.sclk == sd.sclk) the SPI pins must match —
  // otherwise the two begins fight and only one survives. Catch a profile that
  // violates it during bring-up.
#ifdef IT8951_PROBE_DEBUG
  const auto& sd = BoardConfig::ACTIVE.sd;
  if (sd.sclk >= 0 && _sclk == sd.sclk && (_mosi != sd.mosi || _miso != sd.miso) && Serial) {
    Serial.printf("[it8951] WARNING shared-bus pin mismatch: display sclk/mosi/miso=%d/%d/%d sd=%d/%d/%d\n", _sclk,
                  _mosi, _miso, sd.sclk, sd.mosi, sd.miso);
  }
#endif
  _spi.begin(_sclk, _miso, _mosi, -1);  // manual CS toggling

  // Grayscale plane buffers + B/W base snapshot (PSRAM): combined in displayGray().
  const size_t planeBytes = static_cast<size_t>(_fbWb) * _fbH;
  if (!_gLsb) _gLsb = static_cast<uint8_t*>(heap_caps_malloc(planeBytes, MALLOC_CAP_SPIRAM));
  if (!_gMsb) _gMsb = static_cast<uint8_t*>(heap_caps_malloc(planeBytes, MALLOC_CAP_SPIRAM));
  if (!_base) _base = static_cast<uint8_t*>(heap_caps_malloc(planeBytes, MALLOC_CAP_SPIRAM));
  // One 4bpp row for the load loops, sized to this panel (936 B on 1872 px,
  // 480 B on M5Paper's 960). Internal RAM: SPI writeBytes reads it every row.
  // Word-aligned like the DMA chunks: a displayGray4Rows fill may store whole
  // words into the row it is handed.
  if (!_rowBuf) {
    const size_t rowBytes = static_cast<size_t>(_fbWb) * 4;
    _rowBuf = static_cast<uint8_t*>(heap_caps_aligned_alloc(4, rowBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!_rowBuf) _rowBuf = static_cast<uint8_t*>(malloc(rowBytes));  // word-aligned as well
  }
  // DMA bulk write on the second SPI host (no-op if already up, compiled out,
  // or unavailable -- then every load stays on the polled path above).
  initDmaLoad();

  waitReady();
  // SYS_RUN before anything else. deepSleep() leaves the controller in CMD_SLEEP,
  // and on boards whose IT8951 rail is not switched and whose RST is unassigned
  // (eMinimal bench: HAT 5V straight off the devkit) it stays there across the
  // host's deep-sleep wake and even an EN reset — GET_DEV_INFO then reads nothing
  // (the fallback geometry hides it), every wait returns instantly and the glass
  // never changes until a power cycle. Only SYS_RUN wakes it; on a running
  // controller it is a no-op. Waveshare's and M5EPD's init order is the same.
  systemRun();
  getDeviceInfo();
  writeReg(REG_I80CPCR, 0x0001);  // enable host packed write
  setVcom(_cfg.vcomMv);
  // The memory is whatever the last run left (levels, or a bitmap from a run
  // with Bpp1), and so is the engine's bitmap bit: start both at levels. The
  // first load after INIT is always whole-frame, so no stale rows survive.
  _memBitmap = false;
  setBitmapMode(false, /*force=*/true);

  // Resolve rotation: AUTO picks 90° when the panel reports portrait so the
  // landscape framebuffer lands upright.
  _rotation = (_cfg.rotation == IT8951_ROTATE_AUTO) ? (_panelW < _panelH ? 1 : 0) : (_cfg.rotation & 0x03);
  _running = true;

  // Clear to white with the INIT waveform (ignores buffer content).
  displayArea(0, 0, _panelW, _panelH, 0);
  waitDisplayReady();
  if (esp_reset_reason() == ESP_RST_DEEPSLEEP && _cfg.wakeScrub != It8951WakeScrub::None) {
    // Waking with the sleep cover on the glass: INIT alone leaves faint
    // residue of dark art (most visible in dithered fields, and confirmed on
    // eMinimal's glass too). Drive the pigment through a second pass: an
    // explicit white GC16 between two INIT passes (the default), or just a
    // second INIT where the config says that is enough. Cold boots skip the
    // cost. _base is scratch here; the first display() re-snapshots it.
    const bool whitePass =
        _cfg.wakeScrub == It8951WakeScrub::WhiteGc16AndInit || _cfg.wakeScrub == It8951WakeScrub::WhiteGc16;
    if (_base && whitePass) {
      memset(_base, 0xFF, static_cast<size_t>(_fbWb) * _fbH);
      loadImageFull(_base);
      displayArea(0, 0, _panelW, _panelH, _cfg.fullMode);
      waitDisplayReady();
    }
    if (_cfg.wakeScrub != It8951WakeScrub::WhiteGc16) {
      displayArea(0, 0, _panelW, _panelH, 0);
      waitDisplayReady();
    }
  }
  _firstPaintPending = true;
  _memMirrorsBase = false;  // glass is white, memory is whatever it was: first paint reloads
  _memHasGray = false;
  _dirtyValid = false;
  buildGrayTable();
}

// A refresh clears ghosting when it's a Full/Half (the consumer's stronger
// refresh), a wake from standby (fresh image), or the periodic ghost-clear —
// otherwise it's the differential mode. DU/DU4 leave residue that accumulates
// across menu and activity navigation; promoting to GC16 every
// ghostClearInterval refreshes wipes it automatically, like the X3 driver, with
// no firmware involvement. `weight` is this refresh's share of a whole frame in
// 1/256ths: a partial-area menu move counts its box, so a scroll does not flash
// every eight presses the way eight page turns do.
uint16_t It8951Driver::resolveMode(RefreshMode mode, uint16_t differential, uint16_t weight, bool force) {
  const bool clear = force || (mode != RefreshMode::Fast) || !_running || _firstPaintPending ||
                     (_cfg.ghostClearInterval != 0 &&
                      _partialsSinceClear >= static_cast<uint32_t>(_cfg.ghostClearInterval) * FRAME_WEIGHT);
  _firstPaintPending = false;
  _partialsSinceClear = clear ? 0 : _partialsSinceClear + weight;
  _lastClear = clear;
  return clear ? _cfg.fullMode : differential;
}

// Any anti-aliased (non-white, non-black) pixel inside the box, judged from the
// buffered planes against the snapshot: a set base bit is white, a clear one
// with either plane bit set is a gray level.
bool It8951Driver::grayWithin(uint16_t xb0, uint16_t xb1, uint16_t y0, uint16_t y1) const {
  if (!_gLsb || !_gMsb || !_base) return false;
  const uint16_t n = static_cast<uint16_t>(xb1 - xb0 + 1);
  for (uint16_t y = y0; y <= y1; y++) {
    const uint32_t off = static_cast<uint32_t>(y) * _fbWb + xb0;
    const uint8_t* b = _base + off;
    const uint8_t* l = _gLsb + off;
    const uint8_t* m = _gMsb + off;
    for (uint16_t i = 0; i < n; i++) {
      if ((l[i] | m[i]) & ~b[i]) return true;
    }
  }
  return false;
}

// The box now holds plain B/W in the frame memory: forget the planes there so
// a later box over the same place is not mistaken for gray.
void It8951Driver::clearPlanesWithin(uint16_t xb0, uint16_t xb1, uint16_t y0, uint16_t y1) {
  if (!_memHasGray || !_gLsb || !_gMsb) return;
  const size_t n = static_cast<size_t>(xb1 - xb0 + 1);
  for (uint16_t y = y0; y <= y1; y++) {
    const uint32_t off = static_cast<uint32_t>(y) * _fbWb + xb0;
    memset(_gLsb + off, 0, n);
    memset(_gMsb + off, 0, n);
  }
}

void It8951Driver::dirtyUnion(uint16_t xb0, uint16_t xb1, uint16_t y0, uint16_t y1) {
  if (!_dirtyValid) {
    _dirtyXb0 = xb0;
    _dirtyXb1 = xb1;
    _dirtyY0 = y0;
    _dirtyY1 = y1;
    _dirtyValid = true;
    return;
  }
  if (xb0 < _dirtyXb0) _dirtyXb0 = xb0;
  if (xb1 > _dirtyXb1) _dirtyXb1 = xb1;
  if (y0 < _dirtyY0) _dirtyY0 = y0;
  if (y1 > _dirtyY1) _dirtyY1 = y1;
}

void It8951Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  (void)bus;
  (void)prev;  // IT8951 holds the previous frame in its own SRAM
#ifdef IT8951_TIMING_LOG
  const uint32_t tEntry = micros();
  _loadUs = 0;
  _loadBytes = 0;
#endif

  _baseStaged = false;  // a plain B/W display supersedes any base waiting for gray planes

  // When the controller's frame memory mirrors the snapshot, diff the new frame
  // against it: the changed box is all that needs loading, and its share of the
  // frame is what this refresh counts toward the periodic ghost-clear. Needs no
  // rotation between framebuffer and panel (the box is in framebuffer
  // coordinates and DPY_AREA takes panel ones). An empty box means the memory
  // is already right.
  uint16_t xb0 = 0, xb1 = 0, y0 = 0, y1 = 0;
  uint32_t changedBytes = 0;
  const bool canDiff = _memMirrorsBase && _base && fb && _rotation == 0 && _panelW == _fbW && _panelH == _fbH;
  const bool haveBox = canDiff && diffBox(fb, xb0, xb1, y0, y1, changedBytes);
  const bool unchanged = canDiff && !haveBox;
  const bool wholeFrame = !haveBox || (xb0 == 0 && xb1 == _fbWb - 1 && y0 == 0 && y1 == _fbH - 1);
  // "Large" (a screen change) is judged on the area that actually changed, not
  // the bounding box: one diff box spans every change, so a highlight leaving
  // the top of the screen and one arriving lower down (Home's cover row to its
  // menu row), or a highlight plus a footer hint whose label changed (Settings'
  // tab band <-> rows, the "Select" row in a "Toggle" list), made a box over
  // most of the frame -- read as a screen change, and with menu moves behind it
  // (dirty) a full-panel GC16 flash on an ordinary cursor move. It also cost a
  // whole frame toward the periodic clear, so every eighth such move flashed.
  bool largeBox = !haveBox;
  if (haveBox) largeBox = changedBytes * 2 >= static_cast<uint32_t>(_fbWb) * _fbH;
  // Gray under the box: DU is a two-level waveform, and driving anti-aliased
  // text to white with it leaves a shadow of every glyph -- what "back to Home
  // from a book" looked like. The planes are still buffered, so this is known
  // before the load, and such a refresh is promoted to GC16 on the box.
  const bool grayInBox = haveBox && _memHasGray && grayWithin(xb0, xb1, y0, y1);

  // What this refresh is, and what it counts toward the periodic ghost-clear:
  //  - unchanged: nothing driven, counts nothing;
  //  - a partial box over B/W (a menu highlight move): DU on the box, counts
  //    nothing (residue there is mild and the next screen change clears it);
  //  - gray under the box: GC16 on the box; a whole-frame one is a full clear;
  //  - a large box (a screen change): DU, one frame's worth -- or GC16 if
  //    menu moves have left residue behind (dirty), which also resets it.
  bool forceClear = false, boxClear = false;
  uint16_t weight = FRAME_WEIGHT;
  if (unchanged) {
    weight = 0;
  } else if (grayInBox) {
    forceClear = true;
    boxClear = !largeBox;
  } else if (!largeBox) {
    weight = 0;
  } else if (_dirtyValid) {
    forceClear = true;
  }
  // A clear the host asked for, or that the driver owes (first paint, wake from
  // standby), resyncs everything. The others only need the box loaded.
  const bool requestedClear = (mode != RefreshMode::Fast) || !_running || _firstPaintPending;
  // A box-only GC16 leaves residue elsewhere, so it does not count as a clear
  // for the periodic cadence; it just takes the clearing waveform on its box.
  uint16_t dpyMode = resolveMode(mode, _cfg.fastMode, boxClear ? 0 : weight, forceClear && !boxClear);
  if (boxClear) dpyMode = _cfg.fullMode;

#ifdef IT8951_PROBE_DEBUG
  if (Serial)
    Serial.printf("[it8951] display() mode=%d dpyMode=%u n=%lu box=%d large=%d gray=%d dirty=%d\n", (int)mode, dpyMode,
                  (unsigned long)_partialsSinceClear, (int)haveBox, (int)largeBox, (int)grayInBox, (int)_dirtyValid);
#endif

  // Snapshot this B/W frame: the consumer clears the live framebuffer during its
  // grayscale strip pass, so displayGray() needs this copy as the true base.
  if (_base && fb) memcpy(_base, fb, static_cast<size_t>(_fbWb) * _fbH);

  if (boxClear) {
    loadImageBand(fb, y0, y1);
    displayArea(static_cast<uint16_t>(xb0 * 8), y0, static_cast<uint16_t>((xb1 - xb0 + 1) * 8),
                static_cast<uint16_t>(y1 - y0 + 1), dpyMode);
    clearPlanesWithin(0, static_cast<uint16_t>(_fbWb - 1), y0, y1);
  } else if (!_lastClear) {
    if (unchanged) {
      // Nothing changed: drive nothing. A DU pass over an identical frame is
      // not free -- it re-drives dithered fills two-level and leaves them
      // patchy, which is what Home looked like after its second, identical
      // paint following the GC16 out of a book.
    } else if (!wholeFrame) {
      loadImageBand(fb, y0, y1);
      displayArea(static_cast<uint16_t>(xb0 * 8), y0, static_cast<uint16_t>((xb1 - xb0 + 1) * 8),
                  static_cast<uint16_t>(y1 - y0 + 1), dpyMode);
      clearPlanesWithin(0, static_cast<uint16_t>(_fbWb - 1), y0, y1);
      if (!largeBox) dirtyUnion(xb0, xb1, y0, y1);
    } else {
      loadImageFull(fb);
      displayArea(0, 0, _panelW, _panelH, dpyMode);
      _memHasGray = false;
      // Corner-to-corner box but little changed: a partial move like any other.
      if (haveBox && !largeBox) dirtyUnion(xb0, xb1, y0, y1);
    }
  } else if (canDiff && !requestedClear) {
    // A clear with the memory already right outside the box: load the box and
    // GC16 the frame -- no whole-frame push. The periodic clear off a page or
    // a screen change re-drives everything; a clear owed to menu residue
    // could be narrower, but the residue is where the eye is anyway.
    if (haveBox) {
      loadImageBand(fb, y0, y1);
      clearPlanesWithin(0, static_cast<uint16_t>(_fbWb - 1), y0, y1);
      if (wholeFrame) _memHasGray = false;
    }
    displayArea(0, 0, _panelW, _panelH, dpyMode);
    _dirtyValid = false;
  } else {
    loadImageFull(fb);
    displayArea(0, 0, _panelW, _panelH, dpyMode);
    _memHasGray = false;
    _dirtyValid = false;
  }
  waitDisplayReady();
  _memMirrorsBase = _base && fb;
#ifdef IT8951_TIMING_LOG
  // "same frame": the diff found nothing to load; a refresh may still have run
  // (a clear that was owed), which the refresh+wait number shows.
  printTiming(unchanged ? "display (same frame)" : "display", tEntry, dpyMode);
#endif

  if (turnOff) {
    writeCommand(CMD_STANDBY);  // park the controller; next load re-runs SYS_RUN
    _running = false;
  }
}

// Combined base: nothing goes to the panel here. Snapshot the B/W frame and the
// refresh the host wanted for it; displayGray() folds the planes in and drives
// the glass once. If there is no snapshot buffer, fall back to a plain display so
// the page still appears (the gray pass then re-refreshes, as before).
void It8951Driver::displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
  if (!_base || !fb) {
    display(bus, fb, nullptr, fallback, turnOff);
    return;
  }
  memcpy(_base, fb, static_cast<size_t>(_fbWb) * _fbH);
  _memMirrorsBase = false;  // the snapshot is ahead of the frame memory until displayGray() loads it
  _baseStaged = true;
  _stagedMode = fallback;
  _stagedTurnOff = turnOff;
}

void It8951Driver::deepSleep(EpdBus& bus) {
  (void)bus;
  waitDisplayReady();
  writeCommand(CMD_SLEEP);
  _running = false;
}

// --- grayscale -------------------------------------------------------------
// The consumer renders the anti-aliasing LSB/MSB planes and hands them here; we
// buffer them and reconstruct the 16-gray image against the B/W base in
// displayGray(). Full-frame (copyGrayscale*) and per-strip (writeGrayscalePlaneStrip)
// delivery are both supported; the strip path is preferred because it leaves the
// consumer's B/W framebuffer intact, so displayGray() gets the true base buffer.
void It8951Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  (void)bus;
  if (_gLsb && lsb) memcpy(_gLsb, lsb, static_cast<size_t>(_fbWb) * _fbH);
}

void It8951Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  (void)bus;
  if (_gMsb && msb) memcpy(_gMsb, msb, static_cast<size_t>(_fbWb) * _fbH);
}

void It8951Driver::writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                            uint16_t numRows) {
  (void)bus;
  uint8_t* dst = (plane == GrayPlane::Lsb) ? _gLsb : _gMsb;
  if (!dst || !rows) return;
  memcpy(dst + static_cast<uint32_t>(yStart) * _fbWb, rows, static_cast<size_t>(numRows) * _fbWb);
}

void It8951Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut, bool factoryMode) {
  (void)bus;
  (void)lut;
  (void)factoryMode;
  // The consumer's strip-grayscale pass clears the live framebuffer (fb) to 0x00,
  // so use the B/W snapshot captured in display() as the base. Fall back to the
  // passed buffer only if no snapshot exists yet.
  const uint8_t* base = _base ? _base : fb;
#ifdef IT8951_TIMING_LOG
  const uint32_t tEntry = micros();
  _loadUs = 0;
  _loadBytes = 0;
#endif
#ifdef IT8951_PROBE_DEBUG
  if (Serial && base) {
    const uint32_t mid = static_cast<uint32_t>(_fbH / 2) * _fbWb;
    Serial.printf("[it8951] displayGray() snapBase[0,mid]=%02X %02X fb[0]=%02X gLsb=%p gMsb=%p turnOff=%d\n", base[0],
                  base[mid], fb ? fb[0] : 0, _gLsb, _gMsb, turnOff);
  }
#endif
  loadImageGray(base);  // base = snapshot B/W; LSB/MSB already buffered
  _memMirrorsBase = base == _base;
  _memHasGray = _gLsb && _gMsb;
  // Staged base (the normal path): this is the page's only refresh, so it takes
  // the mode the host asked for -- GC16 on its clearing cadence, else DU4, a
  // 4-level differential update that moves only the changed pixels, no flash.
  // Legacy path (a B/W display() already ran): refine the edges with DU4, or
  // GC16 if that pass was a clear so ghosting clears fully.
  uint16_t gmode;
  if (_baseStaged) {
    gmode = resolveMode(_stagedMode, _cfg.grayMode);
    turnOff = turnOff || _stagedTurnOff;
    _baseStaged = false;
  } else {
    gmode = (_lastClear || _firstPaintPending) ? _cfg.fullMode : _cfg.grayMode;
    _firstPaintPending = false;
  }
  displayArea(0, 0, _panelW, _panelH, gmode);
  waitDisplayReady();
#ifdef IT8951_TIMING_LOG
  printTiming("displayGray", tEntry, gmode);
#endif
  // A whole-frame GC16 leaves nothing to clear; a DU4 page leaves the whole
  // frame for the next periodic clear.
  if (gmode == _cfg.fullMode) {
    _dirtyValid = false;
  } else {
    dirtyUnion(0, static_cast<uint16_t>(_fbWb - 1), 0, static_cast<uint16_t>(_fbH - 1));
  }
  if (turnOff) {
    writeCommand(CMD_STANDBY);
    _running = false;
  }
}

// A host-rendered 16-level frame, a row at a time from `fill`: one load, one
// refresh, bookkept like a gray page from displayGray() so the B/W display()
// that follows sees the memory as it is:
//  - a base staged by displayGrayscaleBase() is superseded (this frame is the
//    page); its refresh request and turnOff carry over, the stronger mode wins;
//  - the waveform is resolveMode(mode, grayMode): GC16 for Full/Half, the
//    first paint, a wake from standby or the periodic clear, else DU4 --
//    resolved before the load, so a wake from standby (!_running) clears, as
//    display() does;
//  - _base/_gLsb/_gMsb are re-derived from the rows, so _memMirrorsBase and
//    _memHasGray hold and display()'s diff box, grayWithin() promotion and
//    largeBox logic apply unchanged. Without those buffers, _memMirrorsBase is
//    false and the next display() reloads the whole frame. The host gets the
//    same base in `baseOut` either way;
//  - a DU4 frame joins the dirty region (whole frame); a GC16 one clears it.
bool It8951Driver::displayGray4Rows(EpdBus& bus, Gray4RowFill fill, void* ctx, RefreshMode mode, bool turnOff,
                                    uint8_t* baseOut) {
  (void)bus;
  if (!fill || !_rowBuf) return false;
#ifdef IT8951_TIMING_LOG
  const uint32_t tEntry = micros();
  _loadUs = 0;
  _loadBytes = 0;
#endif
  if (_baseStaged) {
    if (mode == RefreshMode::Fast) mode = _stagedMode;
    turnOff = turnOff || _stagedTurnOff;
    _baseStaged = false;
  }
  const uint16_t gmode = resolveMode(mode, _cfg.grayMode);
  const bool derived = loadImageGray4(fill, ctx, baseOut);
  _memMirrorsBase = derived;
  _memHasGray = derived;
  displayArea(0, 0, _panelW, _panelH, gmode);
  waitDisplayReady();
#ifdef IT8951_TIMING_LOG
  printTiming("displayGray4", tEntry, gmode);
#endif
  if (gmode == _cfg.fullMode) {
    _dirtyValid = false;
  } else {
    dirtyUnion(0, static_cast<uint16_t>(_fbWb - 1), 0, static_cast<uint16_t>(_fbH - 1));
  }
  if (turnOff) {
    writeCommand(CMD_STANDBY);
    _running = false;
  }
  return true;
}

#ifdef IT8951_TIMING_LOG
// One line per refresh: the bytes pushed and how long the push took, the
// waveform that ran and what the rest of the call cost (the refresh itself,
// plus any wait for a refresh still running on entry), and the format the
// frame memory holds afterwards. What the bench reads to put a number on a
// page turn or a menu move; the host's own "Page render" line brackets it.
void It8951Driver::printTiming(const char* what, uint32_t t0Us, uint16_t dpyMode) {
  if (!Serial) return;
  const uint32_t totalUs = micros() - t0Us;
  const uint32_t restUs = totalUs > _loadUs ? totalUs - _loadUs : 0;
  Serial.printf("[it8951] %s: load %lu KB %lu ms (%s), mode %u refresh+wait %lu ms, total %lu ms\n", what,
                (unsigned long)(_loadBytes / 1024), (unsigned long)(_loadUs / 1000), _memBitmap ? "1bpp" : "4bpp",
                dpyMode, (unsigned long)(restUs / 1000), (unsigned long)(totalUs / 1000));
}
#endif

void It8951Driver::cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) {
  // Nothing to re-sync: the IT8951 holds its own frame, and the next display()
  // reloads the framebuffer wholesale. The base B/W frame is left untouched --
  // unless it is still only staged (the gray pass was skipped or failed), in
  // which case put it on the glass as plain B/W rather than lose the page.
  (void)bw;
  if (_baseStaged) {
    _baseStaged = false;
    display(bus, _base, nullptr, _stagedMode, _stagedTurnOff);
  }
}

// Per-board injection mirrors the other drivers: a board wiring the IT8951
// differently (MISO pin, panel VCOM, mount rotation) supplies its own config via
// -DFREEINK_IT8951_CONFIG=yourConfig without editing this driver.
#ifdef FREEINK_IT8951_CONFIG
const It8951Config& FREEINK_IT8951_CONFIG();
static const It8951Config& it8951ActiveConfig() { return FREEINK_IT8951_CONFIG(); }
#elif FREEINK_DEVICE_EMINIMAL
// Supplied by the SDK's BoardEMinimal78 library (add it to lib_deps): MISO 17,
// VCOM from the FPC, and the M841 LUT's mode numbers. The M5Paper default
// below would put MISO on this board's SD D0 and run grayscale through A2.
const It8951Config& eminimal78It8951Config();
static const It8951Config& it8951ActiveConfig() { return eminimal78It8951Config(); }
#else
static const It8951Config& it8951ActiveConfig() { return it8951DefaultConfig(); }
#endif

// Picture pages (XTC reader): after a dark page, a second GC16 over the new
// page, from what the frame memory already holds -- no reload (~0.55 s).
void It8951Driver::repeatLastRefresh(EpdBus& bus) {
  (void)bus;
  if (!_running) return;
  waitDisplayReady();
  displayArea(0, 0, _panelW, _panelH, _cfg.fullMode);
  waitDisplayReady();
}

PanelDriver& it8951Driver() {
  static It8951Driver instance(it8951ActiveConfig());
  return instance;
}

}  // namespace freeink

#endif  // FREEINK_DRIVER_IT8951
