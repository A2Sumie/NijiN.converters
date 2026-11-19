<p align="center">
  <img src="assets/lockup/nijin-lockup-transparent.svg" alt="NijiN" width="420">
</p>

# NijiN.converters

**[English](#english) · [简体中文](#简体中文)**

Clean-room C17 embedded firmware and host verification tooling for electronic lens mount converters (**BORYOZA Hardware Platform**).

> **Source-Only Repository / 实验性质与仅含源码声明**
> - flashing custom firmware to lens mount adapters carries hardware risk; validate builds through host SIL tests before flashing a physical unit and verify bootloaders remain intact.
> - This repository contains **first-party C17 source code, linker scripts, compiled optical lookup data generated from public/measured parameters, and host utilities only** — zero prebuilt `.PUB`/`.bin`/`.elf` images, zero vendor bootloaders, and zero external camera ROM dumps.
> - Only source code, host tests, and documentation live on `main`; prebuilt releases are built locally with `arm-none-eabi-gcc` when needed.

---

## English

### Supported Converters

| Directory | Converter Target | MCU & SoC Architecture | Current Baseline | Focus & Protocol Scope |
|---|---|---|---|---|
| [`se-nz/`](se-nz/) | **Sony E → Nikon Z** (`BORYOZA SE-NZ`) | `HC32F460JEUA` (ARM Cortex-M4F @ `192 MHz`, `512 KiB` Flash, `192 KiB` 5-bank SRAM) | **V2.23** (`LF 2.23` / `17 02 0b 1a`) | Full-time closed-loop `dist_code_x` electronic parfocal (`AF-S`/`AF-C`/`Body MF`/`Barrel MF`), `275`-lens catalog, `57` spectral curves, `25` compressed 3D `CMD 0x24` tables, OIS/IBIS split-axis bridge, POR wake & USB MSC `.PUB` flashing |

### Key Engineering Capabilities (`se-nz/` V2.23)

- **Zero-Calibration Full-Time Electronic Parfocal**: Combines a real-time thin-lens feed-forward model with logarithmic `ID 0x05 dist_code_x` residual closed-loop trimming across `AF-S`, `AF-C`, `Body MF`, and `Barrel MF` modes (`60 Hz` and `120 Hz`), with `ID 0x06 <-> ID 0x05` frame pairing, `C0 -> E0` motor coast protection, and automatic idle `ID 0x04[9] = 0x81` settling release for EVF focus peaking and magnification.
- **Deterministic Real-Time Split (`HC32F460JEUA`)**:
  - `SRAMH` `.ramfunc` ISRs (`SPI3` Z-mount slave, `SPI1` `1 kHz` hotline, `USART1` E-mount DMA, `TMRA4` VSync, `EFM` flash ops) strictly compile with `-mgeneral-regs-only -fno-tree-vectorize -fno-builtin`, enforced at pack time by `pack_pub.py` (`0` VFP instructions and `0` Flash closure leaks).
  - Pure C17 protocol & optical translation core (`se-nz/src/02_core/*.c`) shared verbatim between bare-metal ARM Flash builds and host macOS/Linux ASan+UBSan shared libraries (`libboryoza_core`).
- **On-Board Optical & Spectral Engine**: Includes `275` lens catalog entries, `57` delta-nibble compressed 81-point spectral transmission curves, `25` sparse/Huffman-compressed 3D `CMD 0x24` A/B optical profiles decompressed on the fly into `SRAM2`, plus live `ID 0x05` curve decoding for supported fixed-focal lenses.
- **Fail-Safe Field Diagnostics**: Dual-bank `8 KiB` `Ping/Pong` atomic parameter journal (`0x7C000..0x7FFFF`), `256 × 32 B` CRT/CRC32 ring blackbox (`0x7A000..0x7BFFF`) with wear-out circuit breakers, `4 KiB` `Ret_SRAM` warm-restart cache, `PA0` zero-backfeed `0V` sleep/POR wake recovery, and contiguous FAT12 USB MSC flashing (`se-nz/tools/flash_usb_bor.py`).

### Quick Start (`se-nz`)

#### Prerequisites

- `arm-none-eabi-gcc` / `binutils` (`objcopy`, `objdump`, `size`, `nm`)
- `clang` or `gcc` (host ASan+UBSan SIL build)
- `python3` (3.10+) and GNU `make`

#### Build & Run Host SIL Tests

```sh
# Build MCU firmware (.elf / .bin / .PUB + pack_manifest.json) and host ASan/UBSan shared library
make

# Run static comment/abi/packer checks + host ASan+UBSan closed-loop SIL suite
make test

# Clean build artifacts
make clean
```

All built binaries reside in `se-nz/build/` (`BORYOZA_V223.PUB` is bounded strictly by the physical `[0x00010000, 0x00060000)` `320 KiB` APP Flash partition — `program_end <= 0x00060000`, `<= 327,680 B`).

#### Flash & Read Field Telemetry Over USB MSC

1. Connect the converter in USB bootloader mode so the `BOR` FAT12 volume mounts (`/Volumes/BOR     ` on macOS, or a mounted block directory on Linux):
   ```sh
   python3 se-nz/tools/flash_usb_bor.py --pub se-nz/build/BORYOZA_V223.PUB
   ```
2. Extract and decode Blackbox (`LBA 858..873`) + Journal (`LBA 874..905`) logs:
   ```sh
   python3 se-nz/tools/dump_usb_blackbox.py --out telemetry.json
   ```

See [`docs/README.md`](docs/README.md) for the complete documentation library, [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for memory maps & static gates, and [`docs/FLASHING_AND_DIAGNOSTICS.md`](docs/FLASHING_AND_DIAGNOSTICS.md) for USB MSC flashing & telemetry usage.

#### Complete E→Z Engineering Reference ([`docs/ez-reference/`](docs/ez-reference/))

| Volume | Topic |
|---|---|
| [`HC32F460_SE_NZ_DESIGN_SPEC.md`](docs/ez-reference/HC32F460_SE_NZ_DESIGN_SPEC.md) | `HC32F460JEUA` clean-room firmware design (clocks, DMA, E/Z wire frames, AF/VR/aperture/optics) |
| [`BOOTLOADER_AND_USB_MSC_CONTRACT.md`](docs/ez-reference/BOOTLOADER_AND_USB_MSC_CONTRACT.md) | Stock `64 KiB` Bootloader ABI, `PA9` boot mux, `13`-pin role matrix, and FAT12 MSC writer |
| [`PART1_NIKKOR_Z_FLEET.md`](docs/ez-reference/PART1_NIKKOR_Z_FLEET.md) | Native Nikkor Z RXv2 fleet architecture (`24` firmware packages / `21` lens models) |
| [`PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md`](docs/ez-reference/PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md) | Nikon Z8 `v3.10`/`v3.12` control plane, `1 kHz` hotline, Sony E-mount protocol, and third-party deltas |
| [`PART3_EZ_BRIDGE_AND_ERRATA.md`](docs/ez-reference/PART3_EZ_BRIDGE_AND_ERRATA.md) | E-to-Z bridge translation equations, `Known Bad` catalog, and verified errata |
| [`PART4_PDAF_MATH_AND_CONTROL_OPT.md`](docs/ez-reference/PART4_PDAF_MATH_AND_CONTROL_OPT.md) | Z8 `3.12` rational PDAF solver math, damping/velocity envelopes, M4F opt, and Huffman/spectral codecs |
| [`PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md`](docs/ez-reference/PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md) | Sony E / Nikon Z distance, cam-curve & 5-axis IBIS (`fc_pitch` vs `0xA3`), EXIF `255`/MF bar split, and `0x1D/0x43/0x6B` AF crosswalk |
| [`Z8_SONY_AF_ARCHITECTURE_STUDY.md`](docs/ez-reference/Z8_SONY_AF_ARCHITECTURE_STUDY.md) | Comparative disassembly study: Z8 `3.12` + S-Line primes vs Sony `A1 II` / `A9 III` |
| [`SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md`](docs/ez-reference/SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md) | Master ecosystem protocol & control specification + `275`-lens catalog & `57`-spectral appendix |
| [`SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md`](docs/ez-reference/SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md) | `V2.23` (`LF 2.23`) build manifest, full-time closed-loop electronic parfocal, and verification matrix |

### Repository Layout

```text
.
├── assets/lockup/nijin-lockup-transparent.svg
├── docs/
│   ├── README.md         # Master bilingual documentation index
│   ├── ARCHITECTURE.md
│   ├── FLASHING_AND_DIAGNOSTICS.md
│   └── ez-reference/     # 10-volume complete E->Z protocol, optical, AF math & catalog specs
├── se-nz/
│   ├── Makefile
│   ├── include/          # Register defs, shared C17 state structs, lens DB header
│   ├── ld/               # HC32F460JEUA 512 KiB Flash + 5-bank SRAM linker script
│   ├── src/
│   │   ├── 00_hal/       # Clocks (192 MHz), GPIO pads, TMRA1, SRAM .ramfunc EFM
│   │   ├── 01_transport/ # Z SPI3 DMA, Z SPI1 hotline, E USART1 DMA, TMRA4 VSync
│   │   ├── 02_core/      # MCU + host-shared protocol, AF/parfocal, VR/OIS, optics
│   │   ├── 03_diag_storage/ # 32 B CRC32 blackbox & 128 B dual-sector journal
│   │   └── main.c        # Boot init, PA3 hotplug, TX-busy rollback, main loop
│   ├── tools/            # .PUB packer/verifier, USB MSC flasher, telemetry decoder
│   └── tests/            # Standalone host ASan+UBSan SIL verification suite
├── AGENTS.md
├── LICENSE
├── Makefile
├── NOTICE.md
└── README.md
```

---

## 简体中文

### 支持转接环目标

| 目录 | 转接环目标 | 主控架构与存储 | 当前基线 | 核心协议与特性范围 |
|---|---|---|---|---|
| [`se-nz/`](se-nz/) | **Sony E → Nikon Z**（`BORYOZA SE-NZ`） | `HC32F460JEUA`（ARM Cortex-M4F @ `192 MHz`，`512 KiB` Flash，`192 KiB` 五段 SRAM） | **V2.23**（`LF 2.23` / `17 02 0b 1a`） | 全时闭环 `dist_code_x` 电子齐焦（`AF-S`/`AF-C`/`机身 MF`/`镜身 MF`）、`275` 项镜头目录、`57` 条光谱曲线、`25` 组压缩 3D `CMD 0x24` 光学表、OIS/IBIS 协同、POR 闭环唤醒与 USB MSC `.PUB` 升级 |

### 核心工程特性（`se-nz/` V2.23）

- **零标定全时闭环电子齐焦**：结合薄透镜前馈模型与 `ID 0x05 dist_code_x` 对数分档误差反馈闭环，覆盖 `AF-S`、`AF-C`、`机身 MF` 与`镜身 MF` 全模式及 `60/120 Hz` 刷新率；配备 `ID 0x06 <-> ID 0x05` 采样配对、`C0 -> E0` 滑行保护与 `机身 MF` 主动制动 `0x83 + 0x1C` 逻辑，停稳后自动将 `ID 0x04[9]` 释放回 `0x81` 以支持机身放大对焦与峰值提示。
- **硬实时零浮点分区与软硬同源**：
  - `SRAMH` `.ramfunc` 与全部中断/HAL/传输层函数强制 `-mgeneral-regs-only -fno-tree-vectorize -fno-builtin`，打包工具 `pack_pub.py` 静态扫查零 `VFP` 指令、零 `RAM/EFM` 闭包越界及零 `64-bit` 软除法依赖。
  - 核心逻辑 `se-nz/src/02_core/*.c` 与主机 `macOS`/`Linux` ASan+UBSan 动态库（`libboryoza_core`）共用同一份纯 C17 源码。
- **片上压缩光学与光谱引擎**：内置 `275` 项镜头识别目录、`57` 条 81 点光谱透过率曲线、`25` 组稀疏/Huffman 压缩 `CMD 0x24` A+B 三维光学校正表（运行期双缓冲解压至 `SRAM2`），并支持实时 `ID 0x05` 曲线解码。
- **掉电安全与现场黑匣子诊断**：配备 `Ping/Pong` 双扇区 `128 B` 原子参数日志（`0x7C000..0x7FFFF`）、`256 × 32 B` `CRC32` 飞行黑匣子（`0x7A000..0x7BFFF`）与坏块熔断保护、`4 KiB` `Ret_SRAM` 热启动缓存、`PA0` 零倒灌关机与 `PA9=1` POR 唤醒恢复，以及连续簇 FAT12 USB MSC 刷机工具（`se-nz/tools/flash_usb_bor.py`）。

### 快速上手（`se-nz`）

```sh
# 编译 MCU 固件（.elf / .bin / .PUB 与 pack_manifest.json）及主机 ASan+UBSan 验证库
make

# 运行注释/无软除/打包门禁、工具自检与主机 SIL 闭环测试
make test

# 清理构建产物
make clean
```

构建产物统一生成在 `se-nz/build/` 下；固件体积严格受硬件 APP Flash 分区 `[0x00010000, 0x00060000)`（`320 KiB = 327,680 B`，`program_end <= 0x00060000`）物理边界约束。

更多架构细节、分区映射、USB MSC 连续簇刷写及免提权日志读取步骤，请参阅 [`docs/README.md`](docs/README.md)、[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) 与 [`docs/FLASHING_AND_DIAGNOSTICS.md`](docs/FLASHING_AND_DIAGNOSTICS.md)。

#### 完整 E→Z 实现与协议参考文库（[`docs/ez-reference/`](docs/ez-reference/)）

| 文档 | 内容范围 |
|---|---|
| [`HC32F460_SE_NZ_DESIGN_SPEC.md`](docs/ez-reference/HC32F460_SE_NZ_DESIGN_SPEC.md) | `HC32F460JEUA` E→Z 净室固件完整工程设计规格书（时钟、DMA、双侧线协议与四大核心业务域） |
| [`BOOTLOADER_AND_USB_MSC_CONTRACT.md`](docs/ez-reference/BOOTLOADER_AND_USB_MSC_CONTRACT.md) | 原厂 `64 KiB` Bootloader 交接合同、`PA9` 启动分支、QFN48 管脚归属与 USB MSC FAT12 状态机 |
| [`PART1_NIKKOR_Z_FLEET.md`](docs/ez-reference/PART1_NIKKOR_Z_FLEET.md) | Nikkor Z 原厂 RXv2 镜头固件家族架构（`24` 份解包固件 / `21` 款镜头） |
| [`PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md`](docs/ez-reference/PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md) | Nikon Z8 `v3.10`/`v3.12` 状态机（`164` 描述符/`600` 属性）、`1 kHz SPI1` 热线与 Sony E 口协议差异 |
| [`PART3_EZ_BRIDGE_AND_ERRATA.md`](docs/ez-reference/PART3_EZ_BRIDGE_AND_ERRATA.md) | E→Z 桥接方程、全谱系 `Known Bad` 故障目录与机器码反证历史勘误总汇 |
| [`PART4_PDAF_MATH_AND_CONTROL_OPT.md`](docs/ez-reference/PART4_PDAF_MATH_AND_CONTROL_OPT.md) | Z8 `3.12` 有理式 PDAF 数学拆解、极点诊断、Cortex-M4F 优化与 Canonical Huffman/光谱压缩 |
| [`PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md`](docs/ez-reference/PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md) | Sony E / Nikon Z 全链路距离与倍率、五轴防抖（`fc_pitch` vs `0xA3`）、EXIF `255` 与 MF 标尺分离及三代 AF 指令（`0x1D`/`0x43`/`0x6B`）对照 |
| [`Z8_SONY_AF_ARCHITECTURE_STUDY.md`](docs/ez-reference/Z8_SONY_AF_ARCHITECTURE_STUDY.md) | Z8 `3.12` + 旗舰定焦与 Sony `A1 II` / `A9 III` 双端对焦架构机器码对比研究 |
| [`SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md`](docs/ez-reference/SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md) | SE-NZ 工程生态、控制律总规格书暨 `275` 项镜头目录与 `57` 条光谱附录 |
| [`SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md`](docs/ez-reference/SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md) | `V2.23`（`LF 2.23`）构建指纹、全时闭环电子齐焦设计及全套自动化验证台账 |
