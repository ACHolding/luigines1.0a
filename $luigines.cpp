#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr int kWindowWidth = 600;
constexpr int kWindowHeight = 400;
constexpr int kNesWidth = 256;
constexpr int kNesHeight = 240;

constexpr uint8_t kFlagC = 0x01;
constexpr uint8_t kFlagZ = 0x02;
constexpr uint8_t kFlagI = 0x04;
constexpr uint8_t kFlagD = 0x08;
constexpr uint8_t kFlagB = 0x10;
constexpr uint8_t kFlagU = 0x20;
constexpr uint8_t kFlagV = 0x40;
constexpr uint8_t kFlagN = 0x80;

struct Button {
  SDL_Rect rect{};
  std::string label;
};

struct Cartridge {
  std::vector<uint8_t> prg;
  std::vector<uint8_t> chr;
  bool loaded = false;
  uint16_t mapper = 0;
  int mirrorMode = 0; // 0: horizontal, 1: vertical, 2: single-screen low, 3:
                      // single-screen high
  bool hasChrRam = false;
  bool hasPrgRam = false;
  int prgBankCount16k = 0;
  int chrBankCount8k = 0;
  std::vector<uint8_t> prgRam;

  // Mapper runtime state.
  uint8_t mapper2PrgBank = 0;
  uint8_t mmc1Shift = 0x10;
  uint8_t mmc1Control = 0x0C;
  uint8_t mmc1ChrBank0 = 0;
  uint8_t mmc1ChrBank1 = 0;
  uint8_t mmc1PrgBank = 0;
  uint8_t mmc3BankSelect = 0;
  std::array<uint8_t, 8> mmc3Regs{};
  uint8_t mmc3PrgMode = 0;
  uint8_t mmc3ChrMode = 0;
  // Mapper 3 (CNROM): 8KB CHR bank.
  uint8_t cnromChrBank = 0;
  // Mapper 7 (AxROM): 32KB PRG bank + single-screen mirror select.
  uint8_t axromPrgBank = 0;
  // Mapper 11 (Color Dreams): 32KB PRG + 8KB CHR.
  uint8_t colorDreamsPrg = 0;
  uint8_t colorDreamsChr = 0;
  // Mapper 66 (GxROM): 32KB PRG + 8KB CHR.
  uint8_t gxromPrg = 0;
  uint8_t gxromChr = 0;
  // Mapper 70 (Bandai discrete): 16KB PRG bank at $8000, last bank fixed at
  // $C000; 8KB CHR.
  uint8_t bandai70Prg = 0;
  uint8_t bandai70Chr = 0;
  // Mapper 71 (Camerica/Codemasters): 16KB PRG bank at $8000, last bank fixed
  // at $C000.
  uint8_t camerica71Prg = 0;

  bool loadFromFile(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      std::cerr << "Failed to open ROM: " << path << "\n";
      return false;
    }

    std::array<uint8_t, 16> header{};
    f.read(reinterpret_cast<char *>(header.data()),
           static_cast<std::streamsize>(header.size()));
    if (!f || header[0] != 'N' || header[1] != 'E' || header[2] != 'S' ||
        header[3] != 0x1A) {
      std::cerr << "Invalid iNES header\n";
      return false;
    }

    int prgBanks = header[4];
    int chrBanks = header[5];
    const uint8_t flags6 = header[6];
    const uint8_t flags7 = header[7];
    const bool hasHeaderTailData =
        header[12] || header[13] || header[14] || header[15];
    const bool hasDiskDudeTail =
        header[7] == 'D' && header[8] == 'i' && header[9] == 's' &&
        header[10] == 'k' && header[11] == 'D' && header[12] == 'u' &&
        header[13] == 'd' && header[14] == 'e' && header[15] == '!';
    const bool nes2Header = ((flags7 & 0x0C) == 0x08) && !hasHeaderTailData;
    const uint8_t mapperFlags7 = hasDiskDudeTail ? 0 : flags7;
    mapper = static_cast<uint16_t>((flags6 >> 4) | (mapperFlags7 & 0xF0));
    if (nes2Header) {
      mapper = static_cast<uint16_t>(mapper | ((header[8] & 0x0F) << 8));
      prgBanks = ((header[9] & 0x0F) << 8) | header[4];
      chrBanks = ((header[9] & 0xF0) << 4) | header[5];
    }
    mirrorMode = (flags6 & 0x01) ? 1 : 0;
    if (flags6 & 0x08)
      mirrorMode = 1; // crude four-screen fallback.
    if (!(mapper == 0 || mapper == 1 || mapper == 2 || mapper == 3 ||
          mapper == 4 || mapper == 7 || mapper == 11 || mapper == 66 ||
          mapper == 70 || mapper == 71)) {
      // Fallback: treat unknown mapper as mapper 0 for basic PRG/CHR access.
      // This allows many more commercial/homebrew ROMs to at least load and run
      // basic code. Supported with bankswitching: 0 (NROM), 1 (MMC1), 2
      // (UxROM), 3 (CNROM), 4 (MMC3), 7 (AxROM), 11 (Color Dreams), 66 (GxROM),
      // 70 (Bandai), 71 (Camerica).
      mapper = 0;
    }

    if (flags6 & 0x04) {
      f.seekg(512, std::ios::cur);
    }

    if (prgBanks <= 0) {
      std::cerr << "Missing PRG banks\n";
      return false;
    }

    prg.resize(static_cast<size_t>(prgBanks) * 16384);
    f.read(reinterpret_cast<char *>(prg.data()),
           static_cast<std::streamsize>(prg.size()));
    if (!f) {
      std::cerr << "Failed reading PRG\n";
      return false;
    }

    hasChrRam = (chrBanks == 0);
    hasPrgRam = true;
    prgRam.assign(8192, 0);
    if (chrBanks == 0)
      chrBanks = 1; // CHR RAM fallback.
    chr.resize(static_cast<size_t>(chrBanks) * 8192);
    if (header[5] > 0) {
      f.read(reinterpret_cast<char *>(chr.data()),
             static_cast<std::streamsize>(chr.size()));
      if (!f) {
        std::cerr << "Failed reading CHR\n";
        return false;
      }
    } else {
      std::fill(chr.begin(), chr.end(), 0);
    }

    prgBankCount16k = static_cast<int>(prg.size() / 16384);
    chrBankCount8k = static_cast<int>(chr.size() / 8192);
    resetMapperState();
    loaded = true;
    return true;
  }

  void resetMapperState() {
    mapper2PrgBank = 0;
    mmc1Shift = 0x10;
    mmc1Control = 0x0C;
    mmc1ChrBank0 = 0;
    mmc1ChrBank1 = 0;
    mmc1PrgBank = 0;
    mmc3BankSelect = 0;
    mmc3Regs.fill(0);
    mmc3PrgMode = 0;
    mmc3ChrMode = 0;
    cnromChrBank = 0;
    axromPrgBank = 0;
    colorDreamsPrg = 0;
    colorDreamsChr = 0;
    gxromPrg = 0;
    gxromChr = 0;
    bandai70Prg = 0;
    bandai70Chr = 0;
    camerica71Prg = 0;
    // Mapper 7 power-on: AxROM defaults to single-screen low.
    if (mapper == 7)
      mirrorMode = 2;
    if (hasPrgRam)
      std::fill(prgRam.begin(), prgRam.end(), 0);
  }

  int mapNametableAddress(uint16_t addr) const {
    const uint16_t a = static_cast<uint16_t>((addr - 0x2000) & 0x0FFF);
    const int table = a / 0x0400;
    const int offs = a % 0x0400;
    switch (mirrorMode) {
    case 1: // vertical
      return ((table == 0 || table == 2) ? 0 : 0x400) + offs;
    case 2: // single low
      return offs;
    case 3: // single high
      return 0x400 + offs;
    case 0:
    default: // horizontal
      return ((table == 0 || table == 1) ? 0 : 0x400) + offs;
    }
  }

  uint8_t cpuRead(uint16_t addr) const {
    if (addr >= 0x6000 && addr < 0x8000) {
      if (hasPrgRam && !prgRam.empty())
        return prgRam[addr - 0x6000];
      return 0;
    }
    if (addr < 0x8000 || prg.empty())
      return 0;
    switch (mapper) {
    case 1:
      return cpuReadMmc1(addr);
    case 2:
      return cpuReadMapper2(addr);
    case 4:
      return cpuReadMmc3(addr);
    case 7:
      return cpuRead32kBank(addr, axromPrgBank);
    case 11:
      return cpuRead32kBank(addr, colorDreamsPrg);
    case 66:
      return cpuRead32kBank(addr, gxromPrg);
    case 70:
      return cpuRead16kFixedLast(addr, bandai70Prg);
    case 71:
      return cpuRead16kFixedLast(addr, camerica71Prg);
    case 3:
    case 0:
    default:
      if (prg.size() == 16384)
        return prg[(addr - 0x8000) & 0x3FFF];
      return prg[(addr - 0x8000) & 0x7FFF];
    }
  }

  void cpuWrite(uint16_t addr, uint8_t value) {
    if (addr >= 0x6000 && addr < 0x8000) {
      if (hasPrgRam && !prgRam.empty())
        prgRam[addr - 0x6000] = value;
      return;
    }
    if (addr < 0x8000)
      return;
    switch (mapper) {
    case 1:
      cpuWriteMmc1(addr, value);
      break;
    case 2:
      mapper2PrgBank = static_cast<uint8_t>(value & 0x0F);
      break;
    case 3:
      cnromChrBank = static_cast<uint8_t>(value & 0x03);
      break;
    case 4:
      cpuWriteMmc3(addr, value);
      break;
    case 7:
      axromPrgBank = static_cast<uint8_t>(value & 0x07);
      mirrorMode = (value & 0x10) ? 3 : 2;
      break;
    case 11:
      colorDreamsPrg = static_cast<uint8_t>(value & 0x03);
      colorDreamsChr = static_cast<uint8_t>((value >> 4) & 0x0F);
      break;
    case 66:
      gxromPrg = static_cast<uint8_t>(value & 0x03);
      gxromChr = static_cast<uint8_t>((value >> 4) & 0x03);
      break;
    case 70:
      bandai70Chr = static_cast<uint8_t>(value & 0x0F);
      bandai70Prg = static_cast<uint8_t>((value >> 4) & 0x07);
      break;
    case 71:
      if (addr >= 0xC000) {
        camerica71Prg = static_cast<uint8_t>(value & 0x0F);
      } else if (addr >= 0x9000 && addr < 0xA000) {
        // Codemasters single-screen mirror select.
        mirrorMode = (value & 0x10) ? 3 : 2;
      }
      break;
    default:
      break;
    }
  }

  uint8_t ppuRead(uint16_t addr) const {
    if (chr.empty())
      return 0;
    const uint16_t a = addr & 0x1FFF;
    switch (mapper) {
    case 1:
      return chr[mapMmc1Chr(a) % chr.size()];
    case 4:
      return chr[mapMmc3Chr(a) % chr.size()];
    case 3:
      return chr[(static_cast<size_t>(cnromChrBank) * 8192 + a) % chr.size()];
    case 11:
      return chr[(static_cast<size_t>(colorDreamsChr) * 8192 + a) % chr.size()];
    case 66:
      return chr[(static_cast<size_t>(gxromChr) * 8192 + a) % chr.size()];
    case 70:
      return chr[(static_cast<size_t>(bandai70Chr) * 8192 + a) % chr.size()];
    case 7:
    case 71:
    case 2:
    case 0:
    default:
      return chr[a % chr.size()];
    }
  }

  void ppuWrite(uint16_t addr, uint8_t value) {
    if (!hasChrRam || chr.empty())
      return;
    const uint16_t a = addr & 0x1FFF;
    switch (mapper) {
    case 1:
      chr[mapMmc1Chr(a) % chr.size()] = value;
      break;
    case 4:
      chr[mapMmc3Chr(a) % chr.size()] = value;
      break;
    default:
      chr[a % chr.size()] = value;
      break;
    }
  }

private:
  // 32KB-bank PRG read used by mappers 7 (AxROM), 11 (Color Dreams), 66
  // (GxROM).
  uint8_t cpuRead32kBank(uint16_t addr, uint8_t bank) const {
    const int bankCount32k = std::max(1, static_cast<int>(prg.size() / 32768));
    const int b = bank % bankCount32k;
    const size_t off = static_cast<size_t>(b) * 32768 + (addr - 0x8000);
    return prg[off % prg.size()];
  }

  // 16KB switchable bank at $8000 with last 16KB bank fixed at $C000.
  // Used by mappers 70 (Bandai discrete) and 71 (Camerica/Codemasters).
  uint8_t cpuRead16kFixedLast(uint16_t addr, uint8_t bank) const {
    if (prgBankCount16k <= 0)
      return 0;
    const int lastBank = prgBankCount16k - 1;
    if (addr < 0xC000) {
      const int b = bank % prgBankCount16k;
      const size_t off = static_cast<size_t>(b) * 16384 + (addr - 0x8000);
      return prg[off % prg.size()];
    }
    const size_t off = static_cast<size_t>(lastBank) * 16384 + (addr - 0xC000);
    return prg[off % prg.size()];
  }

  uint8_t cpuReadMapper2(uint16_t addr) const {
    if (prgBankCount16k <= 0)
      return 0;
    const int lastBank = prgBankCount16k - 1;
    if (addr < 0xC000) {
      const int b = mapper2PrgBank % prgBankCount16k;
      const size_t off = static_cast<size_t>(b) * 16384 + (addr - 0x8000);
      return prg[off % prg.size()];
    }
    const size_t off = static_cast<size_t>(lastBank) * 16384 + (addr - 0xC000);
    return prg[off % prg.size()];
  }

  uint8_t cpuReadMmc1(uint16_t addr) const {
    const int prgMode = (mmc1Control >> 2) & 0x03;
    const int chrMode = (mmc1Control >> 4) & 0x01;
    (void)chrMode;
    int bank = 0;
    uint16_t off16 = static_cast<uint16_t>(addr & 0x3FFF);
    if (prgMode <= 1) {
      const int b = (mmc1PrgBank & 0x0E) % std::max(1, prgBankCount16k);
      bank = b + ((addr >= 0xC000) ? 1 : 0);
    } else if (prgMode == 2) {
      bank = (addr < 0xC000) ? 0 : (mmc1PrgBank % std::max(1, prgBankCount16k));
    } else {
      bank = (addr < 0xC000) ? (mmc1PrgBank % std::max(1, prgBankCount16k))
                             : (prgBankCount16k - 1);
    }
    const size_t off = static_cast<size_t>(std::max(0, bank)) * 16384 + off16;
    return prg[off % prg.size()];
  }

  void cpuWriteMmc1(uint16_t addr, uint8_t value) {
    if (value & 0x80) {
      mmc1Shift = 0x10;
      mmc1Control |= 0x0C;
      return;
    }
    const bool complete = (mmc1Shift & 1) != 0;
    mmc1Shift = static_cast<uint8_t>((mmc1Shift >> 1) | ((value & 1) << 4));
    if (!complete)
      return;
    const uint8_t regValue = mmc1Shift & 0x1F;
    if (addr < 0xA000) {
      mmc1Control = regValue;
      switch (mmc1Control & 0x03) {
      case 0:
      case 1:
        mirrorMode = 2;
        break;
      case 2:
        mirrorMode = 1;
        break;
      case 3:
        mirrorMode = 0;
        break;
      }
    } else if (addr < 0xC000) {
      mmc1ChrBank0 = regValue;
    } else if (addr < 0xE000) {
      mmc1ChrBank1 = regValue;
    } else {
      mmc1PrgBank = regValue & 0x0F;
    }
    mmc1Shift = 0x10;
  }

  size_t mapMmc1Chr(uint16_t addr) const {
    const int chrMode = (mmc1Control >> 4) & 0x01;
    if (chrMode == 0) {
      const int bank8k = (mmc1ChrBank0 & 0x1E);
      return static_cast<size_t>(bank8k) * 4096 + addr;
    }
    if (addr < 0x1000) {
      return static_cast<size_t>(mmc1ChrBank0) * 4096 + addr;
    }
    return static_cast<size_t>(mmc1ChrBank1) * 4096 + (addr - 0x1000);
  }

  uint8_t cpuReadMmc3(uint16_t addr) const {
    if (prg.empty())
      return 0;
    const int bank8kCount = std::max(1, static_cast<int>(prg.size() / 8192));
    const int last = bank8kCount - 1;
    const int secondLast = std::max(0, bank8kCount - 2);
    int bank = 0;
    if (addr < 0xA000)
      bank = mmc3PrgMode ? secondLast : (mmc3Regs[6] % bank8kCount);
    else if (addr < 0xC000)
      bank = mmc3Regs[7] % bank8kCount;
    else if (addr < 0xE000)
      bank = mmc3PrgMode ? (mmc3Regs[6] % bank8kCount) : secondLast;
    else
      bank = last;
    const size_t off = static_cast<size_t>(bank) * 8192 + (addr & 0x1FFF);
    return prg[off % prg.size()];
  }

  void cpuWriteMmc3(uint16_t addr, uint8_t value) {
    if ((addr & 1) == 0) {
      if (addr >= 0x8000 && addr <= 0x9FFE) {
        mmc3BankSelect = value & 0x07;
        mmc3PrgMode = (value >> 6) & 1;
        mmc3ChrMode = (value >> 7) & 1;
      } else if (addr >= 0xA000 && addr <= 0xBFFE) {
        mirrorMode = (value & 1) ? 1 : 0;
      }
    } else {
      if (addr >= 0x8001 && addr <= 0x9FFF) {
        mmc3Regs[mmc3BankSelect & 0x07] = value;
      }
    }
  }

  size_t mapMmc3Chr(uint16_t addr) const {
    const int bank1kCount = std::max(1, static_cast<int>(chr.size() / 1024));
    auto bankOff = [&](int bank, uint16_t local) -> size_t {
      return static_cast<size_t>(bank % bank1kCount) * 1024 + local;
    };
    const uint16_t a = addr & 0x1FFF;
    if (!mmc3ChrMode) {
      if (a < 0x0800)
        return bankOff(mmc3Regs[0] & 0xFE, a);
      if (a < 0x1000)
        return bankOff(mmc3Regs[1] & 0xFE, static_cast<uint16_t>(a - 0x0800));
      if (a < 0x1400)
        return bankOff(mmc3Regs[2], static_cast<uint16_t>(a - 0x1000));
      if (a < 0x1800)
        return bankOff(mmc3Regs[3], static_cast<uint16_t>(a - 0x1400));
      if (a < 0x1C00)
        return bankOff(mmc3Regs[4], static_cast<uint16_t>(a - 0x1800));
      return bankOff(mmc3Regs[5], static_cast<uint16_t>(a - 0x1C00));
    }
    if (a < 0x0400)
      return bankOff(mmc3Regs[2], a);
    if (a < 0x0800)
      return bankOff(mmc3Regs[3], static_cast<uint16_t>(a - 0x0400));
    if (a < 0x0C00)
      return bankOff(mmc3Regs[4], static_cast<uint16_t>(a - 0x0800));
    if (a < 0x1000)
      return bankOff(mmc3Regs[5], static_cast<uint16_t>(a - 0x0C00));
    if (a < 0x1800)
      return bankOff(mmc3Regs[0] & 0xFE, static_cast<uint16_t>(a - 0x1000));
    return bankOff(mmc3Regs[1] & 0xFE, static_cast<uint16_t>(a - 0x1800));
  }
};

class Ppu2C02;

class Bus {
public:
  explicit Bus(Cartridge &c);

  void attachPpu(Ppu2C02 *p);
  uint8_t read(uint16_t addr);
  void write(uint16_t addr, uint8_t value);

  void setController(uint8_t state) {
    controllerState = state;
    if (controllerStrobe)
      controllerShift = controllerState;
  }

  bool pollNmi();
  void reset();

private:
  Cartridge &cart;
  std::array<uint8_t, 2048> ram{};
  Ppu2C02 *ppu = nullptr;
  uint8_t controllerState = 0;
  uint8_t controllerShift = 0;
  bool controllerStrobe = false;
};

class Cpu6502 {
public:
  explicit Cpu6502(Bus &b) : bus(b) {}

  void reset() {
    A = X = Y = 0;
    S = 0xFD;
    P = kFlagU | kFlagI;
    const uint8_t lo = bus.read(0xFFFC);
    const uint8_t hi = bus.read(0xFFFD);
    PC = static_cast<uint16_t>((hi << 8) | lo);
    if (PC < 0x8000)
      PC = 0x8000;
  }

  int step() {
    if (bus.pollNmi()) {
      nmi();
      return 7;
    }
    const uint8_t op = fetch8();
    switch (op) {
    case 0xA9:
      A = fetch8();
      setZN(A);
      return 2; // LDA #imm
    case 0xA5:
      A = read(zp());
      setZN(A);
      return 3; // LDA zp
    case 0xB5:
      A = read(zpX());
      setZN(A);
      return 4; // LDA zp,X
    case 0xAD:
      A = read(abs());
      setZN(A);
      return 4; // LDA abs
    case 0xBD: {
      bool p = false;
      A = read(absX(p));
      setZN(A);
      return 4 + pagePenalty(p);
    }
    case 0xB9: {
      bool p = false;
      A = read(absY(p));
      setZN(A);
      return 4 + pagePenalty(p);
    }
    case 0xA1:
      A = read(indX());
      setZN(A);
      return 6; // LDA (zp,X)
    case 0xB1: {
      bool p = false;
      A = read(indY(p));
      setZN(A);
      return 5 + pagePenalty(p);
    }

    case 0x85:
      write(zp(), A);
      return 3; // STA zp
    case 0x95:
      write(zpX(), A);
      return 4; // STA zp,X
    case 0x8D:
      write(abs(), A);
      return 4; // STA abs
    case 0x9D:
      write(absX(), A);
      return 5; // STA abs,X
    case 0x99:
      write(absY(), A);
      return 5; // STA abs,Y
    case 0x81:
      write(indX(), A);
      return 6; // STA (zp,X)
    case 0x91:
      write(indY(), A);
      return 6; // STA (zp),Y

    case 0xA2:
      X = fetch8();
      setZN(X);
      return 2; // LDX #imm
    case 0xA6:
      X = read(zp());
      setZN(X);
      return 3; // LDX zp
    case 0xB6:
      X = read(zpY());
      setZN(X);
      return 4; // LDX zp,Y
    case 0xAE:
      X = read(abs());
      setZN(X);
      return 4; // LDX abs
    case 0xBE: {
      bool p = false;
      X = read(absY(p));
      setZN(X);
      return 4 + pagePenalty(p);
    }
    case 0x86:
      write(zp(), X);
      return 3; // STX zp
    case 0x96:
      write(zpY(), X);
      return 4; // STX zp,Y
    case 0x8E:
      write(abs(), X);
      return 4; // STX abs

    case 0xA0:
      Y = fetch8();
      setZN(Y);
      return 2; // LDY #imm
    case 0xA4:
      Y = read(zp());
      setZN(Y);
      return 3; // LDY zp
    case 0xB4:
      Y = read(zpX());
      setZN(Y);
      return 4; // LDY zp,X
    case 0xAC:
      Y = read(abs());
      setZN(Y);
      return 4; // LDY abs
    case 0xBC: {
      bool p = false;
      Y = read(absX(p));
      setZN(Y);
      return 4 + pagePenalty(p);
    }
    case 0x84:
      write(zp(), Y);
      return 3; // STY zp
    case 0x94:
      write(zpX(), Y);
      return 4; // STY zp,X
    case 0x8C:
      write(abs(), Y);
      return 4; // STY abs

    case 0xE8:
      ++X;
      setZN(X);
      return 2; // INX
    case 0xC8:
      ++Y;
      setZN(Y);
      return 2; // INY
    case 0xCA:
      --X;
      setZN(X);
      return 2; // DEX
    case 0x88:
      --Y;
      setZN(Y);
      return 2; // DEY
    case 0x9A:
      S = X;
      return 2; // TXS
    case 0xBA:
      X = S;
      setZN(X);
      return 2; // TSX
    case 0x4C:
      PC = abs();
      return 3; // JMP abs
    case 0x6C:
      PC = ind();
      return 5; // JMP (ind)
    case 0x20:
      jsr(abs());
      return 6; // JSR
    case 0x60:
      rts();
      return 6; // RTS
    case 0x40:
      rti();
      return 6; // RTI
    case 0x69:
      adc(fetch8());
      return 2; // ADC #imm
    case 0x65:
      adc(read(zp()));
      return 3; // ADC zp
    case 0x75:
      adc(read(zpX()));
      return 4; // ADC zp,X
    case 0x6D:
      adc(read(abs()));
      return 4; // ADC abs
    case 0x7D: {
      bool p = false;
      adc(read(absX(p)));
      return 4 + pagePenalty(p);
    }
    case 0x79: {
      bool p = false;
      adc(read(absY(p)));
      return 4 + pagePenalty(p);
    }
    case 0x61:
      adc(read(indX()));
      return 6; // ADC (zp,X)
    case 0x71: {
      bool p = false;
      adc(read(indY(p)));
      return 5 + pagePenalty(p);
    }
    case 0xE9:
      sbc(fetch8());
      return 2; // SBC #imm
    case 0xEB:
      sbc(fetch8());
      return 2; // SBC #imm unofficial alias
    case 0xE5:
      sbc(read(zp()));
      return 3; // SBC zp
    case 0xF5:
      sbc(read(zpX()));
      return 4; // SBC zp,X
    case 0xED:
      sbc(read(abs()));
      return 4; // SBC abs
    case 0xFD: {
      bool p = false;
      sbc(read(absX(p)));
      return 4 + pagePenalty(p);
    }
    case 0xF9: {
      bool p = false;
      sbc(read(absY(p)));
      return 4 + pagePenalty(p);
    }
    case 0xE1:
      sbc(read(indX()));
      return 6; // SBC (zp,X)
    case 0xF1: {
      bool p = false;
      sbc(read(indY(p)));
      return 5 + pagePenalty(p);
    }
    case 0x18:
      P &= ~kFlagC;
      return 2; // CLC
    case 0x38:
      P |= kFlagC;
      return 2; // SEC
    case 0x58:
      P &= ~kFlagI;
      return 2; // CLI
    case 0x78:
      P |= kFlagI;
      return 2; // SEI
    case 0xD8:
      P &= ~kFlagD;
      return 2; // CLD
    case 0xF8:
      P |= kFlagD;
      return 2; // SED
    case 0xB8:
      P &= ~kFlagV;
      return 2; // CLV
    case 0xEA:
      return 2; // NOP
    case 0x00:
      brk();
      return 7; // BRK
    case 0x48:
      push(A);
      return 3; // PHA
    case 0x68:
      A = pull();
      setZN(A);
      return 4; // PLA
    case 0x08:
      push(P | kFlagB | kFlagU);
      return 3; // PHP
    case 0x28:
      P = (pull() | kFlagU) & ~kFlagB;
      return 4; // PLP
    case 0xAA:
      X = A;
      setZN(X);
      return 2; // TAX
    case 0x8A:
      A = X;
      setZN(A);
      return 2; // TXA
    case 0xA8:
      Y = A;
      setZN(Y);
      return 2; // TAY
    case 0x98:
      A = Y;
      setZN(A);
      return 2; // TYA
    case 0xD0:
      return branch((P & kFlagZ) == 0); // BNE
    case 0xF0:
      return branch((P & kFlagZ) != 0); // BEQ
    case 0x10:
      return branch((P & kFlagN) == 0); // BPL
    case 0x30:
      return branch((P & kFlagN) != 0); // BMI
    case 0x90:
      return branch((P & kFlagC) == 0); // BCC
    case 0xB0:
      return branch((P & kFlagC) != 0); // BCS
    case 0x70:
      return branch((P & kFlagV) != 0); // BVS
    case 0x50:
      return branch((P & kFlagV) == 0); // BVC

    case 0x29:
      andA(fetch8());
      return 2; // AND #imm
    case 0x25:
      andA(read(zp()));
      return 3; // AND zp
    case 0x35:
      andA(read(zpX()));
      return 4; // AND zp,X
    case 0x2D:
      andA(read(abs()));
      return 4; // AND abs
    case 0x3D: {
      bool p = false;
      andA(read(absX(p)));
      return 4 + pagePenalty(p);
    }
    case 0x39: {
      bool p = false;
      andA(read(absY(p)));
      return 4 + pagePenalty(p);
    }
    case 0x21:
      andA(read(indX()));
      return 6; // AND (zp,X)
    case 0x31: {
      bool p = false;
      andA(read(indY(p)));
      return 5 + pagePenalty(p);
    }

    case 0x09:
      ora(fetch8());
      return 2; // ORA #imm
    case 0x05:
      ora(read(zp()));
      return 3; // ORA zp
    case 0x15:
      ora(read(zpX()));
      return 4; // ORA zp,X
    case 0x0D:
      ora(read(abs()));
      return 4; // ORA abs
    case 0x1D: {
      bool p = false;
      ora(read(absX(p)));
      return 4 + pagePenalty(p);
    }
    case 0x19: {
      bool p = false;
      ora(read(absY(p)));
      return 4 + pagePenalty(p);
    }
    case 0x01:
      ora(read(indX()));
      return 6; // ORA (zp,X)
    case 0x11: {
      bool p = false;
      ora(read(indY(p)));
      return 5 + pagePenalty(p);
    }

    case 0x49:
      eor(fetch8());
      return 2; // EOR #imm
    case 0x45:
      eor(read(zp()));
      return 3; // EOR zp
    case 0x55:
      eor(read(zpX()));
      return 4; // EOR zp,X
    case 0x4D:
      eor(read(abs()));
      return 4; // EOR abs
    case 0x5D: {
      bool p = false;
      eor(read(absX(p)));
      return 4 + pagePenalty(p);
    }
    case 0x59: {
      bool p = false;
      eor(read(absY(p)));
      return 4 + pagePenalty(p);
    }
    case 0x41:
      eor(read(indX()));
      return 6; // EOR (zp,X)
    case 0x51: {
      bool p = false;
      eor(read(indY(p)));
      return 5 + pagePenalty(p);
    }

    case 0xC9:
      cmp(fetch8());
      return 2; // CMP #imm
    case 0xC5:
      cmp(read(zp()));
      return 3; // CMP zp
    case 0xD5:
      cmp(read(zpX()));
      return 4; // CMP zp,X
    case 0xCD:
      cmp(read(abs()));
      return 4; // CMP abs
    case 0xDD: {
      bool p = false;
      cmp(read(absX(p)));
      return 4 + pagePenalty(p);
    }
    case 0xD9: {
      bool p = false;
      cmp(read(absY(p)));
      return 4 + pagePenalty(p);
    }
    case 0xC1:
      cmp(read(indX()));
      return 6; // CMP (zp,X)
    case 0xD1: {
      bool p = false;
      cmp(read(indY(p)));
      return 5 + pagePenalty(p);
    }
    case 0xE0:
      cpx(fetch8());
      return 2; // CPX #imm
    case 0xE4:
      cpx(read(zp()));
      return 3; // CPX zp
    case 0xEC:
      cpx(read(abs()));
      return 4; // CPX abs
    case 0xC0:
      cpy(fetch8());
      return 2; // CPY #imm
    case 0xC4:
      cpy(read(zp()));
      return 3; // CPY zp
    case 0xCC:
      cpy(read(abs()));
      return 4; // CPY abs
    case 0xE6:
      inc(zp());
      return 5; // INC zp
    case 0xF6:
      inc(zpX());
      return 6; // INC zp,X
    case 0xEE:
      inc(abs());
      return 6; // INC abs
    case 0xFE:
      inc(absX());
      return 7; // INC abs,X
    case 0xC6:
      dec(zp());
      return 5; // DEC zp
    case 0xD6:
      dec(zpX());
      return 6; // DEC zp,X
    case 0xCE:
      dec(abs());
      return 6; // DEC abs
    case 0xDE:
      dec(absX());
      return 7; // DEC abs,X
    case 0x0A:
      aslA();
      return 2; // ASL A
    case 0x06:
      asl(zp());
      return 5; // ASL zp
    case 0x16:
      asl(zpX());
      return 6; // ASL zp,X
    case 0x0E:
      asl(abs());
      return 6; // ASL abs
    case 0x1E:
      asl(absX());
      return 7; // ASL abs,X
    case 0x4A:
      lsrA();
      return 2; // LSR A
    case 0x46:
      lsr(zp());
      return 5; // LSR zp
    case 0x56:
      lsr(zpX());
      return 6; // LSR zp,X
    case 0x4E:
      lsr(abs());
      return 6; // LSR abs
    case 0x5E:
      lsr(absX());
      return 7; // LSR abs,X
    case 0x2A:
      rolA();
      return 2; // ROL A
    case 0x26:
      rol(zp());
      return 5; // ROL zp
    case 0x36:
      rol(zpX());
      return 6; // ROL zp,X
    case 0x2E:
      rol(abs());
      return 6; // ROL abs
    case 0x3E:
      rol(absX());
      return 7; // ROL abs,X
    case 0x6A:
      rorA();
      return 2; // ROR A
    case 0x66:
      ror(zp());
      return 5; // ROR zp
    case 0x76:
      ror(zpX());
      return 6; // ROR zp,X
    case 0x6E:
      ror(abs());
      return 6; // ROR abs
    case 0x7E:
      ror(absX());
      return 7; // ROR abs,X
    case 0x24:
      bit(read(zp()));
      return 3; // BIT zp
    case 0x2C:
      bit(read(abs()));
      return 4; // BIT abs

    // ---- Unofficial / illegal opcodes (used by commercial ROMs) ----
    // 1-byte implied NOPs.
    case 0x1A:
    case 0x3A:
    case 0x5A:
    case 0x7A:
    case 0xDA:
    case 0xFA:
      return 2;
    // 2-byte immediate NOPs.
    case 0x80:
    case 0x82:
    case 0x89:
    case 0xC2:
    case 0xE2:
      fetch8();
      return 2;
    // 2-byte zp NOPs.
    case 0x04:
    case 0x44:
    case 0x64:
      fetch8();
      return 3;
    // 2-byte zp,X NOPs.
    case 0x14:
    case 0x34:
    case 0x54:
    case 0x74:
    case 0xD4:
    case 0xF4:
      fetch8();
      return 4;
    // 3-byte abs NOP.
    case 0x0C:
      fetch16();
      return 4;
    // 3-byte abs,X NOPs.
    case 0x1C:
    case 0x3C:
    case 0x5C:
    case 0x7C:
    case 0xDC:
    case 0xFC:
      fetch16();
      return 4;

    // LAX (load A and X from same address).
    case 0xA3:
      lax(indX());
      return 6; // LAX (zp,X)
    case 0xA7:
      lax(zp());
      return 3; // LAX zp
    case 0xAF:
      lax(abs());
      return 4; // LAX abs
    case 0xB3:
      lax(indY());
      return 5; // LAX (zp),Y
    case 0xB7:
      lax(zpY());
      return 4; // LAX zp,Y
    case 0xBF:
      lax(absY());
      return 4; // LAX abs,Y

    // SAX (store A & X).
    case 0x83:
      sax(indX());
      return 6; // SAX (zp,X)
    case 0x87:
      sax(zp());
      return 3; // SAX zp
    case 0x8F:
      sax(abs());
      return 4; // SAX abs
    case 0x97:
      sax(zpY());
      return 4; // SAX zp,Y

    // DCP (DEC + CMP A).
    case 0xC3:
      dcp(indX());
      return 8;
    case 0xC7:
      dcp(zp());
      return 5;
    case 0xCF:
      dcp(abs());
      return 6;
    case 0xD3:
      dcp(indY());
      return 8;
    case 0xD7:
      dcp(zpX());
      return 6;
    case 0xDB:
      dcp(absY());
      return 7;
    case 0xDF:
      dcp(absX());
      return 7;

    // ISB / ISC (INC + SBC).
    case 0xE3:
      isb(indX());
      return 8;
    case 0xE7:
      isb(zp());
      return 5;
    case 0xEF:
      isb(abs());
      return 6;
    case 0xF3:
      isb(indY());
      return 8;
    case 0xF7:
      isb(zpX());
      return 6;
    case 0xFB:
      isb(absY());
      return 7;
    case 0xFF:
      isb(absX());
      return 7;

    // SLO / RLA / SRE / RRA are common in commercial titles.
    case 0x03:
      slo(indX());
      return 8;
    case 0x07:
      slo(zp());
      return 5;
    case 0x0F:
      slo(abs());
      return 6;
    case 0x13:
      slo(indY());
      return 8;
    case 0x17:
      slo(zpX());
      return 6;
    case 0x1B:
      slo(absY());
      return 7;
    case 0x1F:
      slo(absX());
      return 7;
    case 0x23:
      rla(indX());
      return 8;
    case 0x27:
      rla(zp());
      return 5;
    case 0x2F:
      rla(abs());
      return 6;
    case 0x33:
      rla(indY());
      return 8;
    case 0x37:
      rla(zpX());
      return 6;
    case 0x3B:
      rla(absY());
      return 7;
    case 0x3F:
      rla(absX());
      return 7;
    case 0x43:
      sre(indX());
      return 8;
    case 0x47:
      sre(zp());
      return 5;
    case 0x4F:
      sre(abs());
      return 6;
    case 0x53:
      sre(indY());
      return 8;
    case 0x57:
      sre(zpX());
      return 6;
    case 0x5B:
      sre(absY());
      return 7;
    case 0x5F:
      sre(absX());
      return 7;
    case 0x63:
      rra(indX());
      return 8;
    case 0x67:
      rra(zp());
      return 5;
    case 0x6F:
      rra(abs());
      return 6;
    case 0x73:
      rra(indY());
      return 8;
    case 0x77:
      rra(zpX());
      return 6;
    case 0x7B:
      rra(absY());
      return 7;
    case 0x7F:
      rra(absX());
      return 7;

    default:
      fetch8();
      return 2; // Unknown: skip operand to avoid desync
    }
  }

private:
  Bus &bus;
  uint8_t A = 0;
  uint8_t X = 0;
  uint8_t Y = 0;
  uint8_t P = kFlagU;
  uint8_t S = 0xFD;
  uint16_t PC = 0x8000;

  uint8_t read(uint16_t a) { return bus.read(a); }
  void write(uint16_t a, uint8_t v) { bus.write(a, v); }
  uint8_t fetch8() { return read(PC++); }
  uint16_t fetch16() {
    const uint8_t lo = fetch8();
    return static_cast<uint16_t>(lo | (fetch8() << 8));
  }
  uint16_t zp() { return fetch8(); }
  uint16_t zpX() { return static_cast<uint8_t>(fetch8() + X); }
  uint16_t zpY() { return static_cast<uint8_t>(fetch8() + Y); }
  uint16_t abs() { return fetch16(); }
  uint16_t absX() { return static_cast<uint16_t>(fetch16() + X); }
  uint16_t absY() { return static_cast<uint16_t>(fetch16() + Y); }
  uint16_t indX() {
    const uint8_t ptr = static_cast<uint8_t>(fetch8() + X);
    return static_cast<uint16_t>(read(ptr) |
                                 (read(static_cast<uint8_t>(ptr + 1)) << 8));
  }
  uint16_t indY() {
    const uint8_t ptr = fetch8();
    const uint16_t base = static_cast<uint16_t>(
        read(ptr) | (read(static_cast<uint8_t>(ptr + 1)) << 8));
    return static_cast<uint16_t>(base + Y);
  }
  uint16_t ind() {
    const uint16_t ptr = fetch16();
    const uint8_t lo = read(ptr);
    const uint8_t hi =
        read(static_cast<uint16_t>((ptr & 0xFF00) | ((ptr + 1) & 0x00FF)));
    return static_cast<uint16_t>(lo | (hi << 8));
  }

  static int pagePenalty(bool crossed) { return crossed ? 1 : 0; }
  uint16_t absX(bool &crossed) {
    const uint16_t base = fetch16();
    const uint16_t addr = static_cast<uint16_t>(base + X);
    crossed = (base & 0xFF00) != (addr & 0xFF00);
    return addr;
  }
  uint16_t absY(bool &crossed) {
    const uint16_t base = fetch16();
    const uint16_t addr = static_cast<uint16_t>(base + Y);
    crossed = (base & 0xFF00) != (addr & 0xFF00);
    return addr;
  }
  uint16_t indY(bool &crossed) {
    const uint8_t ptr = fetch8();
    const uint16_t base = static_cast<uint16_t>(
        read(ptr) | (read(static_cast<uint8_t>(ptr + 1)) << 8));
    const uint16_t addr = static_cast<uint16_t>(base + Y);
    crossed = (base & 0xFF00) != (addr & 0xFF00);
    return addr;
  }

  void setZN(uint8_t v) {
    if (v == 0)
      P |= kFlagZ;
    else
      P &= ~kFlagZ;
    if (v & 0x80)
      P |= kFlagN;
    else
      P &= ~kFlagN;
  }

  void push(uint8_t v) {
    write(static_cast<uint16_t>(0x0100 | S), v);
    --S;
  }

  uint8_t pull() {
    ++S;
    return read(static_cast<uint16_t>(0x0100 | S));
  }

  void jsr(uint16_t addr) {
    const uint16_t ret = static_cast<uint16_t>(PC - 1);
    push(static_cast<uint8_t>((ret >> 8) & 0xFF));
    push(static_cast<uint8_t>(ret & 0xFF));
    PC = addr;
  }

  void rts() {
    const uint8_t lo = pull();
    const uint8_t hi = pull();
    PC = static_cast<uint16_t>(((hi << 8) | lo) + 1);
  }

  void rti() {
    P = (pull() | kFlagU) & ~kFlagB;
    const uint8_t lo = pull();
    const uint8_t hi = pull();
    PC = static_cast<uint16_t>((hi << 8) | lo);
  }

  void brk() {
    const uint16_t ret = static_cast<uint16_t>(PC + 1);
    push(static_cast<uint8_t>((ret >> 8) & 0xFF));
    push(static_cast<uint8_t>(ret & 0xFF));
    push(static_cast<uint8_t>((P | kFlagB | kFlagU)));
    P |= kFlagI;
    const uint8_t lo = read(0xFFFE);
    const uint8_t hi = read(0xFFFF);
    PC = static_cast<uint16_t>((hi << 8) | lo);
  }

  void adc(uint8_t v) {
    const uint16_t sum = static_cast<uint16_t>(A) + v + ((P & kFlagC) ? 1 : 0);
    const uint8_t result = static_cast<uint8_t>(sum & 0xFF);
    if (sum > 0xFF)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    if ((~(A ^ v) & (A ^ result) & 0x80) != 0)
      P |= kFlagV;
    else
      P &= ~kFlagV;
    A = result;
    setZN(A);
  }

  void sbc(uint8_t v) { adc(static_cast<uint8_t>(~v)); }

  void andA(uint8_t v) {
    A &= v;
    setZN(A);
  }

  void ora(uint8_t v) {
    A |= v;
    setZN(A);
  }

  void eor(uint8_t v) {
    A ^= v;
    setZN(A);
  }

  void cmp(uint8_t v) {
    const uint16_t tmp = static_cast<uint16_t>(A) - v;
    if (A >= v)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    setZN(static_cast<uint8_t>(tmp & 0xFF));
  }

  void cpx(uint8_t v) {
    const uint16_t tmp = static_cast<uint16_t>(X) - v;
    if (X >= v)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    setZN(static_cast<uint8_t>(tmp & 0xFF));
  }

  void cpy(uint8_t v) {
    const uint16_t tmp = static_cast<uint16_t>(Y) - v;
    if (Y >= v)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    setZN(static_cast<uint8_t>(tmp & 0xFF));
  }

  void inc(uint16_t a) {
    uint8_t v = read(a);
    ++v;
    write(a, v);
    setZN(v);
  }

  void dec(uint16_t a) {
    uint8_t v = read(a);
    --v;
    write(a, v);
    setZN(v);
  }

  void aslA() {
    if (A & 0x80)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    A <<= 1;
    setZN(A);
  }

  void asl(uint16_t a) {
    uint8_t v = read(a);
    if (v & 0x80)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    v <<= 1;
    write(a, v);
    setZN(v);
  }

  void lsrA() {
    if (A & 0x01)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    A >>= 1;
    setZN(A);
  }

  void lsr(uint16_t a) {
    uint8_t v = read(a);
    if (v & 0x01)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    v >>= 1;
    write(a, v);
    setZN(v);
  }

  void rolA() {
    const uint8_t carry = (P & kFlagC) ? 1 : 0;
    if (A & 0x80)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    A = static_cast<uint8_t>((A << 1) | carry);
    setZN(A);
  }

  void rol(uint16_t a) {
    uint8_t v = read(a);
    const uint8_t carry = (P & kFlagC) ? 1 : 0;
    if (v & 0x80)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    v = static_cast<uint8_t>((v << 1) | carry);
    write(a, v);
    setZN(v);
  }

  void rorA() {
    const uint8_t carry = (P & kFlagC) ? 0x80 : 0;
    if (A & 0x01)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    A = static_cast<uint8_t>((A >> 1) | carry);
    setZN(A);
  }

  void ror(uint16_t a) {
    uint8_t v = read(a);
    const uint8_t carry = (P & kFlagC) ? 0x80 : 0;
    if (v & 0x01)
      P |= kFlagC;
    else
      P &= ~kFlagC;
    v = static_cast<uint8_t>((v >> 1) | carry);
    write(a, v);
    setZN(v);
  }

  void bit(uint8_t v) {
    if (v & 0x80)
      P |= kFlagN;
    else
      P &= ~kFlagN;
    if (v & 0x40)
      P |= kFlagV;
    else
      P &= ~kFlagV;
    if ((A & v) == 0)
      P |= kFlagZ;
    else
      P &= ~kFlagZ;
  }

  int branch(bool cond) {
    const int8_t offset = static_cast<int8_t>(fetch8());
    if (!cond)
      return 2;
    const uint16_t oldPC = PC;
    PC = static_cast<uint16_t>(PC + offset);
    return ((oldPC & 0xFF00) != (PC & 0xFF00)) ? 4 : 3;
  }

  // Unofficial: LAX = LDA + LDX from same address.
  void lax(uint16_t a) {
    A = read(a);
    X = A;
    setZN(A);
  }

  // Unofficial: SAX = store (A & X) to memory; flags unaffected.
  void sax(uint16_t a) { write(a, static_cast<uint8_t>(A & X)); }

  // Unofficial: DCP = DEC then CMP A against the new value.
  void dcp(uint16_t a) {
    uint8_t v = read(a);
    --v;
    write(a, v);
    cmp(v);
  }

  // Unofficial: ISB/ISC = INC then SBC the new value from A.
  void isb(uint16_t a) {
    uint8_t v = read(a);
    ++v;
    write(a, v);
    sbc(v);
  }

  void slo(uint16_t a) {
    asl(a);
    ora(read(a));
  }

  void rla(uint16_t a) {
    rol(a);
    andA(read(a));
  }

  void sre(uint16_t a) {
    lsr(a);
    eor(read(a));
  }

  void rra(uint16_t a) {
    ror(a);
    adc(read(a));
  }

  void nmi() {
    push(static_cast<uint8_t>((PC >> 8) & 0xFF));
    push(static_cast<uint8_t>(PC & 0xFF));
    push(static_cast<uint8_t>((P & ~kFlagB) | kFlagU));
    P |= kFlagI;
    const uint8_t lo = read(0xFFFA);
    const uint8_t hi = read(0xFFFB);
    PC = static_cast<uint16_t>((hi << 8) | lo);
    if (PC < 0x8000)
      PC = 0x8000;
  }
};

class Ppu2C02 {
public:
  explicit Ppu2C02(Cartridge &c) : cart(c) { frame.fill(0xFF000000); }

  void reset() {
    scanline = 0;
    cycle = 0;
    ppuctrl = 0;
    ppumask = 0;
    ppustatus = 0;
    oamAddr = 0;
    addrLatch = false;
    vramAddr = 0;
    tempAddr = 0;
    fineX = 0;
    dataBuffer = 0;
    ciram.fill(0);
    paletteRam.fill(0);
    oam.fill(0);
    paletteRam[0x00] = 0x0F; // power-on black instead of gray.
    paletteRam[0x04] = 0x01;
    paletteRam[0x08] = 0x11;
    paletteRam[0x0C] = 0x21;
  }

  void clock() {
    const bool visibleOrPrerender =
        (scanline >= 0 && scanline < 240) || scanline == 261;
    const bool bgEnabled = (ppumask & 0x08) != 0;
    const bool spriteEnabled = (ppumask & 0x10) != 0;
    const bool rendering = bgEnabled || spriteEnabled;

    if (visibleOrPrerender && rendering) {
      if (scanline == 261 && cycle >= 280 && cycle <= 304) {
        // Vertical bits transfer t -> v (pre-render).
        vramAddr =
            static_cast<uint16_t>((vramAddr & 0x841F) | (tempAddr & 0x7BE0));
      }
      if (cycle == 257) {
        // Horizontal bits transfer t -> v (visible + pre-render).
        vramAddr =
            static_cast<uint16_t>((vramAddr & 0xFBE0) | (tempAddr & 0x041F));
      }
      if (cycle > 0 && cycle <= 256) {
        if ((cycle % 8) == 0) {
          incrementCoarseX();
        }
        if (cycle == 256) {
          incrementY();
        }
      }
    }

    ++cycle;
    if (scanline == 241 && cycle == 1) {
      ppustatus |= 0x80; // VBlank
      if (ppuctrl & 0x80) {
        nmiPending = true;
      }
    }
    if (scanline == 261 && cycle == 1) {
      ppustatus &= ~0x80;
      ppustatus &= ~0x40; // Sprite 0 hit clear.
      nmiPending = false;
    }
    if (cycle >= 341) {
      cycle = 0;
      ++scanline;
      if (scanline >= 262) {
        scanline = 0;
        renderFrame();
      }
    }
  }

  void cpuWriteRegister(uint16_t addr, uint8_t value) {
    switch (addr & 0x2007) {
    case 0x2000:
      ppuctrl = value;
      tempAddr =
          static_cast<uint16_t>((tempAddr & 0xF3FF) | ((value & 0x03) << 10));
      break;
    case 0x2001:
      ppumask = value;
      break;
    case 0x2003:
      oamAddr = value;
      break;
    case 0x2004:
      oam[oamAddr++] = value;
      break;
    case 0x2005:
      if (!addrLatch) {
        fineX = value & 0x07;
        tempAddr = static_cast<uint16_t>((tempAddr & 0xFFE0) | (value >> 3));
      } else {
        tempAddr =
            static_cast<uint16_t>((tempAddr & 0x8FFF) | ((value & 0x07) << 12));
        tempAddr =
            static_cast<uint16_t>((tempAddr & 0xFC1F) | ((value & 0xF8) << 2));
      }
      addrLatch = !addrLatch;
      break;
    case 0x2006:
      if (!addrLatch) {
        tempAddr =
            static_cast<uint16_t>((tempAddr & 0x00FF) | ((value & 0x3F) << 8));
      } else {
        tempAddr = static_cast<uint16_t>((tempAddr & 0xFF00) | value);
        vramAddr = tempAddr;
      }
      addrLatch = !addrLatch;
      break;
    case 0x2007:
      ppuWrite(vramAddr, value);
      vramAddr = static_cast<uint16_t>(vramAddr + ((ppuctrl & 0x04) ? 32 : 1));
      break;
    default:
      break;
    }
  }

  uint8_t cpuReadRegister(uint16_t addr) {
    switch (addr & 0x2007) {
    case 0x2002: {
      uint8_t v = ppustatus;
      ppustatus &= ~0x80;
      addrLatch = false;
      nmiPending = false;
      return v;
    }
    case 0x2004:
      return oam[oamAddr];
    case 0x2007: {
      const uint8_t v = ppuRead(vramAddr);
      uint8_t out = dataBuffer;
      dataBuffer = v;
      if ((vramAddr & 0x3FFF) >= 0x3F00)
        out = v;
      vramAddr = static_cast<uint16_t>(vramAddr + ((ppuctrl & 0x04) ? 32 : 1));
      return out;
    }
    default:
      return 0;
    }
  }

  const std::array<uint32_t, kNesWidth * kNesHeight> &getFrame() const {
    return frame;
  }

  bool pullNmi() {
    const bool v = nmiPending;
    nmiPending = false;
    return v;
  }

  // OAM DMA: write one byte to OAM[oamAddr] and post-increment oamAddr (wraps
  // at 256). Called 256 times by the bus when CPU writes to $4014.
  void oamDmaWrite(uint8_t value) { oam[oamAddr++] = value; }

private:
  Cartridge &cart;
  std::array<uint32_t, kNesWidth * kNesHeight> frame{};
  std::array<uint8_t, 2048> ciram{};
  std::array<uint8_t, 32> paletteRam{};
  std::array<uint8_t, 256> oam{};
  int scanline = 0;
  int cycle = 0;
  uint8_t ppuctrl = 0;
  uint8_t ppumask = 0;
  uint8_t ppustatus = 0;
  uint8_t oamAddr = 0;
  bool addrLatch = false;
  uint16_t vramAddr = 0;
  uint16_t tempAddr = 0;
  uint8_t fineX = 0;
  uint8_t dataBuffer = 0;
  bool nmiPending = false;

  static uint32_t pal(uint8_t idx) {
    static constexpr std::array<uint32_t, 64> p = {
        0xFF545454, 0xFF001E74, 0xFF081090, 0xFF300088, 0xFF440064, 0xFF5C0030,
        0xFF540400, 0xFF3C1800, 0xFF202A00, 0xFF083A00, 0xFF004000, 0xFF003C00,
        0xFF00323C, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF989698, 0xFF084CC4,
        0xFF3032EC, 0xFF5C1EE4, 0xFF8814B0, 0xFFA01464, 0xFF982220, 0xFF783C00,
        0xFF545A00, 0xFF287200, 0xFF087C00, 0xFF007628, 0xFF006678, 0xFF000000,
        0xFF000000, 0xFF000000, 0xFFECEEEC, 0xFF4C9AEC, 0xFF787CEC, 0xFFB062EC,
        0xFFE454EC, 0xFFEC58B4, 0xFFEC6A64, 0xFFD48820, 0xFFA0AA00, 0xFF74C400,
        0xFF4CD020, 0xFF38CC6C, 0xFF38B4CC, 0xFF3C3C3C, 0xFF000000, 0xFF000000,
        0xFFECEEEC, 0xFFA8CCEC, 0xFFBCBCEC, 0xFFD4B2EC, 0xFFECAEEC, 0xFFECAED4,
        0xFFECB4B0, 0xFFE4C490, 0xFFCCD278, 0xFFB4DE78, 0xFFA8E290, 0xFF98E2B4,
        0xFFA0D6E4, 0xFFA0A2A0, 0xFF000000, 0xFF000000};
    return p[idx & 0x3F];
  }

  uint16_t mirrorNametableAddress(uint16_t addr) const {
    return static_cast<uint16_t>(cart.mapNametableAddress(addr) & 0x07FF);
  }

  bool renderingEnabled() const { return (ppumask & 0x18) != 0; }

  void incrementCoarseX() {
    if ((vramAddr & 0x001F) == 31) {
      vramAddr &= ~0x001F;
      vramAddr ^= 0x0400;
    } else {
      ++vramAddr;
    }
  }

  void incrementY() {
    if ((vramAddr & 0x7000) != 0x7000) {
      vramAddr += 0x1000;
    } else {
      vramAddr &= ~0x7000;
      int y = (vramAddr & 0x03E0) >> 5;
      if (y == 29) {
        y = 0;
        vramAddr ^= 0x0800;
      } else if (y == 31) {
        y = 0;
      } else {
        ++y;
      }
      vramAddr = static_cast<uint16_t>((vramAddr & ~0x03E0) | (y << 5));
    }
  }

  uint8_t ppuRead(uint16_t addr) const {
    const uint16_t a = addr & 0x3FFF;
    if (a < 0x2000) {
      return cart.ppuRead(a);
    }
    if (a < 0x3F00) {
      return ciram[mirrorNametableAddress(a)];
    }
    const uint16_t p = static_cast<uint16_t>(0x3F00 + ((a - 0x3F00) & 0x1F));
    uint16_t idx = p & 0x1F;
    if (idx == 0x10)
      idx = 0x00;
    if (idx == 0x14)
      idx = 0x04;
    if (idx == 0x18)
      idx = 0x08;
    if (idx == 0x1C)
      idx = 0x0C;
    return paletteRam[idx];
  }

  void ppuWrite(uint16_t addr, uint8_t value) {
    const uint16_t a = addr & 0x3FFF;
    if (a < 0x2000) {
      cart.ppuWrite(a, value);
      return;
    }
    if (a < 0x3F00) {
      ciram[mirrorNametableAddress(a)] = value;
      return;
    }
    uint16_t idx = static_cast<uint16_t>((a - 0x3F00) & 0x1F);
    if (idx == 0x10)
      idx = 0x00;
    if (idx == 0x14)
      idx = 0x04;
    if (idx == 0x18)
      idx = 0x08;
    if (idx == 0x1C)
      idx = 0x0C;
    paletteRam[idx] = value;
  }

  void renderFrame() {
    if (!cart.loaded || cart.chr.empty()) {
      for (int y = 0; y < kNesHeight; ++y) {
        for (int x = 0; x < kNesWidth; ++x) {
          frame[y * kNesWidth + x] = ((x ^ y) & 8) ? 0xFF0A1A30 : 0xFF050A12;
        }
      }
      return;
    }

    const uint16_t baseNt =
        static_cast<uint16_t>(0x2000 + ((tempAddr >> 10) & 0x03) * 0x0400);
    const uint16_t bgPatternBase = (ppuctrl & 0x10) ? 0x1000 : 0x0000;
    const int scrollX = ((tempAddr & 0x001F) << 3) | fineX;
    const int scrollY =
        (((tempAddr >> 5) & 0x001F) << 3) | ((tempAddr >> 12) & 0x07);
    uint8_t universalBg = ppuRead(0x3F00);
    if (universalBg == 0)
      universalBg = 0x0F;

    // Fast background path for common games: nametable + attribute + pattern
    // decode.
    for (int y = 0; y < kNesHeight; ++y) {
      for (int x = 0; x < kNesWidth; ++x) {
        const int sx = (x + scrollX) & 0x1FF;
        const int sy = (y + scrollY) % 240;
        const int tableX = (sx >> 8) & 1;
        const int tableY = 0;
        const uint16_t ntBase = static_cast<uint16_t>(
            baseNt ^ (tableX ? 0x0400 : 0) ^ (tableY ? 0x0800 : 0));
        const int tileX = (sx >> 3) & 31;
        const int tileY = (sy >> 3) & 29;
        const int tx = sx & 7;
        const int ty = sy & 7;
        const uint16_t ntAddr =
            static_cast<uint16_t>(ntBase + tileY * 32 + tileX);
        const uint8_t tileIndex = ppuRead(ntAddr);

        const uint16_t attrAddr = static_cast<uint16_t>(
            ntBase + 0x03C0 + (tileY / 4) * 8 + (tileX / 4));
        const uint8_t attr = ppuRead(attrAddr);
        const int shift = ((tileY & 0x02) << 1) | (tileX & 0x02);
        const uint8_t paletteSel = static_cast<uint8_t>((attr >> shift) & 0x03);

        const uint16_t pattAddr =
            static_cast<uint16_t>(bgPatternBase + tileIndex * 16 + ty);
        const uint8_t lo = ppuRead(pattAddr);
        const uint8_t hi = ppuRead(static_cast<uint16_t>(pattAddr + 8));
        const uint8_t b0 = (lo >> (7 - tx)) & 1;
        const uint8_t b1 = (hi >> (7 - tx)) & 1;
        const uint8_t col = static_cast<uint8_t>((b1 << 1) | b0);
        uint8_t bgNesColor = universalBg;
        const bool bgOpaque = col != 0;
        if (bgOpaque) {
          const uint16_t palAddr =
              static_cast<uint16_t>(0x3F00 + (paletteSel * 4 + col));
          bgNesColor = ppuRead(palAddr);
        }

        uint8_t spriteNesColor = 0;
        bool spriteVisible = false;
        bool spriteBehindBg = false;
        bool sprite0HitHere = false;
        sampleSpritePixel(x, y, bgOpaque, spriteNesColor, spriteVisible,
                          spriteBehindBg, sprite0HitHere);

        if (sprite0HitHere && x < 255) {
          ppustatus |= 0x40;
        }

        uint8_t finalColor = bgNesColor;
        if (spriteVisible && (!spriteBehindBg || !bgOpaque)) {
          finalColor = spriteNesColor;
        }
        frame[y * kNesWidth + x] = pal(finalColor);
      }
    }
  }

  void sampleSpritePixel(int x, int y, bool bgOpaque, uint8_t &nesColorOut,
                         bool &visibleOut, bool &behindBgOut,
                         bool &sprite0HitOut) {
    visibleOut = false;
    behindBgOut = false;
    sprite0HitOut = false;
    nesColorOut = 0;
    if ((ppumask & 0x10) == 0)
      return;

    const bool sprite8x16 = (ppuctrl & 0x20) != 0;
    const int spriteHeight = sprite8x16 ? 16 : 8;

    for (int i = 0; i < 64; ++i) {
      const int base = i * 4;
      const int sy = oam[base] + 1;
      const uint8_t tile = oam[base + 1];
      const uint8_t attr = oam[base + 2];
      const int sx = oam[base + 3];
      if (x < sx || x >= (sx + 8) || y < sy || y >= (sy + spriteHeight))
        continue;

      int row = y - sy;
      if (attr & 0x80)
        row = spriteHeight - 1 - row;
      int col = x - sx;
      if (attr & 0x40)
        col = 7 - col;

      uint16_t spritePatternBase = (ppuctrl & 0x08) ? 0x1000 : 0x0000;
      uint8_t spriteTile = tile;
      if (sprite8x16) {
        spritePatternBase =
            static_cast<uint16_t>((tile & 0x01) ? 0x1000 : 0x0000);
        spriteTile = static_cast<uint8_t>(tile & 0xFE);
        if (row >= 8) {
          spriteTile = static_cast<uint8_t>(spriteTile + 1);
          row -= 8;
        }
      }

      const uint16_t addr =
          static_cast<uint16_t>(spritePatternBase + spriteTile * 16 + row);
      const uint8_t lo = ppuRead(addr);
      const uint8_t hi = ppuRead(static_cast<uint16_t>(addr + 8));
      const uint8_t b0 = (lo >> (7 - col)) & 1;
      const uint8_t b1 = (hi >> (7 - col)) & 1;
      const uint8_t pix = static_cast<uint8_t>((b1 << 1) | b0);
      if (pix == 0)
        continue;

      const uint8_t palSel = static_cast<uint8_t>(attr & 0x03);
      const uint16_t palAddr = static_cast<uint16_t>(0x3F10 + palSel * 4 + pix);
      nesColorOut = ppuRead(palAddr);
      behindBgOut = (attr & 0x20) != 0;
      visibleOut = true;
      sprite0HitOut = (i == 0) && bgOpaque;
      return;
    }
  }
};

Bus::Bus(Cartridge &c) : cart(c) {}

void Bus::attachPpu(Ppu2C02 *p) { ppu = p; }

bool Bus::pollNmi() { return ppu ? ppu->pullNmi() : false; }

uint8_t Bus::read(uint16_t addr) {
  if (addr < 0x2000) {
    return ram[addr & 0x07FF];
  }
  if (addr < 0x4000) {
    return ppu ? ppu->cpuReadRegister(
                     static_cast<uint16_t>(0x2000 | (addr & 0x0007)))
               : 0;
  }
  if (addr == 0x4016) {
    const uint8_t bit = controllerShift & 1;
    controllerShift >>= 1;
    return bit | 0x40;
  }
  if (addr >= 0x4000 && addr < 0x4020) {
    // APU / I/O area (stubs for now)
    if (addr == 0x4016) {
      const uint8_t bit = controllerShift & 1;
      controllerShift >>= 1;
      return bit | 0x40;
    }
    return 0; // open bus / APU read stub
  }
  if (addr >= 0x8000) {
    return cart.cpuRead(addr);
  }
  return 0;
}

void Bus::write(uint16_t addr, uint8_t value) {
  if (addr < 0x2000) {
    ram[addr & 0x07FF] = value;
    return;
  }
  if (addr < 0x4000) {
    if (ppu)
      ppu->cpuWriteRegister(static_cast<uint16_t>(0x2000 | (addr & 0x0007)),
                            value);
    return;
  }
  if (addr == 0x4014) {
    // OAM DMA: copy 256 bytes from CPU page (value<<8) into PPU OAM.
    if (ppu) {
      const uint16_t base = static_cast<uint16_t>(value) << 8;
      for (int i = 0; i < 256; ++i) {
        ppu->oamDmaWrite(read(static_cast<uint16_t>(base + i)));
      }
    }
    return;
  }
  if (addr == 0x4016) {
    if ((value & 1) != 0) {
      controllerStrobe = true;
    } else if (controllerStrobe) {
      controllerStrobe = false;
      controllerShift = controllerState;
    }
    return;
  }
  if (addr >= 0x4000 && addr < 0x4020) {
    // APU writes ignored for now
    return;
  }
  if (addr >= 0x8000) {
    cart.cpuWrite(addr, value);
  }
}

void Bus::reset() {
  ram.fill(0);
  controllerState = 0;
  controllerShift = 0;
  controllerStrobe = false;
}

class NesCore {
public:
  NesCore() : bus(cart), cpu(bus), ppu(cart) { bus.attachPpu(&ppu); }

  bool loadRom(const std::string &path) {
    if (!cart.loadFromFile(path))
      return false;
    bus.reset();
    cpu.reset();
    ppu.reset();
    romPath = path;
    return true;
  }

  void reset() {
    cart.resetMapperState();
    bus.reset();
    cpu.reset();
    ppu.reset();
  }

  void runFrame(bool running) {
    if (!running || !cart.loaded)
      return;
    int cpuCycles = 0;
    while (cpuCycles < 29780) {
      const int c = cpu.step();
      cpuCycles += c;
      for (int i = 0; i < c * 3; ++i)
        ppu.clock();
    }
  }

  void setControllerFromKeyboard(const Uint8 *keys) {
    uint8_t st = 0;
    st |= keys[SDL_SCANCODE_X] ? (1 << 0) : 0;      // A
    st |= keys[SDL_SCANCODE_Z] ? (1 << 1) : 0;      // B
    st |= keys[SDL_SCANCODE_RSHIFT] ? (1 << 2) : 0; // Select
    st |= keys[SDL_SCANCODE_RETURN] ? (1 << 3) : 0; // Start
    st |= keys[SDL_SCANCODE_UP] ? (1 << 4) : 0;
    st |= keys[SDL_SCANCODE_DOWN] ? (1 << 5) : 0;
    st |= keys[SDL_SCANCODE_LEFT] ? (1 << 6) : 0;
    st |= keys[SDL_SCANCODE_RIGHT] ? (1 << 7) : 0;
    bus.setController(st);
  }

  const auto &frame() const { return ppu.getFrame(); }
  bool loaded() const { return cart.loaded; }
  const std::string &path() const { return romPath; }
  int mapperId() const { return cart.mapper; }

private:
  Cartridge cart;
  Bus bus;
  Cpu6502 cpu;
  Ppu2C02 ppu;
  std::string romPath;
};

bool inside(const SDL_Rect &r, int x, int y) {
  return x >= r.x && y >= r.y && x < (r.x + r.w) && y < (r.y + r.h);
}

void drawText(SDL_Renderer *ren, TTF_Font *font, const std::string &text, int x,
              int y, SDL_Color color) {
  SDL_Surface *surf = TTF_RenderUTF8_Blended(font, text.c_str(), color);
  if (!surf)
    return;
  SDL_Texture *tex = SDL_CreateTextureFromSurface(ren, surf);
  if (!tex) {
    SDL_FreeSurface(surf);
    return;
  }
  SDL_Rect dst{x, y, surf->w, surf->h};
  SDL_RenderCopy(ren, tex, nullptr, &dst);
  SDL_DestroyTexture(tex);
  SDL_FreeSurface(surf);
}

void drawButton(SDL_Renderer *ren, TTF_Font *font, const Button &b) {
  SDL_SetRenderDrawColor(ren, 5, 8, 16, 255);
  SDL_RenderFillRect(ren, &b.rect);
  SDL_SetRenderDrawColor(ren, 48, 104, 204, 255);
  SDL_RenderDrawRect(ren, &b.rect);
  drawText(ren, font, b.label, b.rect.x + 10, b.rect.y + 10,
           SDL_Color{126, 188, 255, 255});
}

std::string pickRomOnMac() {
#if defined(__APPLE__)
  const char *cmd = "osascript -e 'set f to choose file with prompt \"Select a "
                    "NES ROM\" of type {\"nes\"}' "
                    "-e 'POSIX path of f'";
  FILE *p = popen(cmd, "r");
  if (!p)
    return {};
  char buf[2048];
  std::string out;
  while (fgets(buf, sizeof(buf), p) != nullptr)
    out += buf;
  const int rc = pclose(p);
  if (rc != 0)
    return {};
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
    out.pop_back();
  return out;
#else
  return {};
#endif
}

} // namespace

int main(int argc, char **argv) {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER) != 0) {
    std::cerr << "SDL init failed: " << SDL_GetError() << "\n";
    return 1;
  }
  if (TTF_Init() != 0) {
    std::cerr << "TTF init failed: " << TTF_GetError() << "\n";
    SDL_Quit();
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow(
      "Luigines", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      kWindowWidth, kWindowHeight, SDL_WINDOW_SHOWN);
  if (!window) {
    std::cerr << "Window creation failed\n";
    TTF_Quit();
    SDL_Quit();
    return 1;
  }

  SDL_Renderer *ren = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!ren) {
    std::cerr << "Renderer creation failed\n";
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    return 1;
  }

  TTF_Font *font =
      TTF_OpenFont("/System/Library/Fonts/Supplemental/Arial Unicode.ttf", 14);
  if (!font) {
    std::cerr << "Font load failed: " << TTF_GetError() << "\n";
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    return 1;
  }

  SDL_Texture *frameTex =
      SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                        SDL_TEXTUREACCESS_STREAMING, kNesWidth, kNesHeight);
  if (!frameTex) {
    std::cerr << "Frame texture creation failed\n";
    TTF_CloseFont(font);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    return 1;
  }

  NesCore nes;
  bool running = false;
  std::string status = "Load a .nes ROM (button, drag/drop, or argv)";
  if (argc > 1) {
    if (nes.loadRom(argv[1])) {
      running = true;
      status = "Loaded m" + std::to_string(nes.mapperId()) + ": " + nes.path();
    } else {
      status = "ROM load failed from argv";
    }
  }

  const std::vector<Button> buttons = {{{420, 44, 160, 44}, "Load ROM"},
                                       {{420, 98, 160, 44}, "Run/Pause"},
                                       {{420, 152, 160, 44}, "Reset"},
                                       {{420, 206, 160, 44}, "Quit"}};

  const SDL_Rect nesView{16, 44, 392, 336};
  bool quit = false;

  while (!quit) {
    SDL_Event e{};
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT)
        quit = true;
      if (e.type == SDL_DROPFILE) {
        std::string dropped = e.drop.file ? e.drop.file : "";
        if (!dropped.empty() && nes.loadRom(dropped)) {
          running = true;
          status = "Loaded m" + std::to_string(nes.mapperId()) + ": " + dropped;
        } else {
          status = "Failed loading dropped ROM";
        }
        if (e.drop.file)
          SDL_free(e.drop.file);
      }
      if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_o &&
          ((e.key.keysym.mod & KMOD_GUI) || (e.key.keysym.mod & KMOD_CTRL))) {
        const std::string path = pickRomOnMac();
        if (!path.empty() && nes.loadRom(path)) {
          running = true;
          status = "Loaded m" + std::to_string(nes.mapperId()) + ": " + path;
        } else {
          status = "Open ROM canceled or failed";
        }
      }
      if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
        const int mx = e.button.x;
        const int my = e.button.y;
        if (inside(buttons[0].rect, mx, my)) {
          const std::string path = pickRomOnMac();
          if (!path.empty() && nes.loadRom(path)) {
            running = true;
            status = "Loaded m" + std::to_string(nes.mapperId()) + ": " + path;
          } else {
            status = "Open ROM canceled or failed";
          }
        } else if (inside(buttons[1].rect, mx, my)) {
          running = !running;
          status = running ? "Running" : "Paused";
        } else if (inside(buttons[2].rect, mx, my)) {
          if (nes.loaded()) {
            nes.reset();
            status = "Core reset";
          } else {
            status = "No ROM loaded";
          }
        } else if (inside(buttons[3].rect, mx, my)) {
          quit = true;
        }
      }
    }

    const Uint8 *keys = SDL_GetKeyboardState(nullptr);
    nes.setControllerFromKeyboard(keys);
    nes.runFrame(running);

    SDL_UpdateTexture(frameTex, nullptr, nes.frame().data(),
                      kNesWidth * static_cast<int>(sizeof(uint32_t)));

    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    SDL_RenderClear(ren);

    SDL_SetRenderDrawColor(ren, 8, 14, 28, 255);
    SDL_Rect side{410, 0, 190, 400};
    SDL_RenderFillRect(ren, &side);

    SDL_SetRenderDrawColor(ren, 32, 76, 160, 255);
    SDL_RenderDrawRect(ren, &nesView);
    SDL_Rect inner{nesView.x + 2, nesView.y + 2, nesView.w - 4, nesView.h - 4};
    SDL_RenderCopy(ren, frameTex, nullptr, &inner);

    drawText(ren, font, "Luigines", 16, 14,
             SDL_Color{115, 178, 255, 255});
    drawText(ren, font, "A:X B:Z Sel:RShift Start:Enter", 16, 384,
             SDL_Color{84, 142, 216, 255});
    drawText(ren, font, "DPad: Arrow Keys", 420, 270,
             SDL_Color{80, 140, 220, 255});
    drawText(ren, font, "Cmd/Ctrl+O open ROM", 420, 292,
             SDL_Color{80, 140, 220, 255});
    drawText(ren, font, running ? "State: Running" : "State: Paused", 420, 314,
             SDL_Color{108, 170, 248, 255});
    drawText(ren, font, nes.loaded() ? "ROM: loaded" : "ROM: none", 420, 336,
             SDL_Color{108, 170, 248, 255});
    drawText(ren, font, status, 16, 366, SDL_Color{88, 149, 228, 255});

    for (const auto &b : buttons)
      drawButton(ren, font, b);

    SDL_RenderPresent(ren);
  }

  SDL_DestroyTexture(frameTex);
  TTF_CloseFont(font);
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(window);
  TTF_Quit();
  SDL_Quit();
  return 0;
}
