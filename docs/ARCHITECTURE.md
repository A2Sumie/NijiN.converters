# SE-NZ Firmware Architecture (`HC32F460JEUA`, V2.24)

## 1. MCU Memory & Partition Layout (`se-nz/ld/hc32f460jeua_boryoza.ld`)

### 1.1 Internal Flash (`512 KiB`: `0x00000000 .. 0x0007FFFF`)

| Address Range | Role | Hard Constraint |
|---|---|---|
| `0x00000000 .. 0x0000FFFF` (`64 KiB`) | Stock USB MSC Bootloader + OTP / UID Block (`0x04200`) | Untouched by `.PUB` upgrades (`LBA >= 10`) |
| `0x00010000 .. 0x00010FFF` (`4 KiB`) | `.isr_vector` & early reset stub | `[0] = 0x1FFFFF00` (MSP), `[1] = 0x00010281` (`Reset_Handler`) |
| `0x00011000 .. 0x00011003` (`4 B`) | `.readme_ver` version stamp | Fixed LMA `0x00011000` (`{0x18, 0x02, 0x0B, 0x1A}` = `LF 2.24`) |
| `0x00011008 .. <0x00060000` (`<= 320 KiB`) | APP `.text`, `.rodata` (`276` lens DB, `58` spectral, `92` 3D `CMD 0x24`), `.ramfunc` & `.data` load image | Absolute hardware/partition cap: `__flash_image_end <= 0x0005FFE0`, `program_end <= 0x00060000` |
| `0x00060000 .. 0x00077FFF` (`96 KiB`) | USB MSC FAT12 filesystem & bootloader scratch window | Strictly isolated from APP Flash & diagnostic sectors |
| `0x00078000 .. 0x00079FFF` (`8 KiB`) | Active Ping/Pong Journal `JNL_PONG` (`LBA 842..857`, `64 × 128 B`) | Alternates atomically with `JNL_A` (`0x7C000`) |
| `0x0007A000 .. 0x0007BFFF` (`8 KiB`) | Blackbox ring buffer (`LBA 858..873`, `256 × 32 B`) | Power-loss tear-safe scan + flash fault circuit breaker |
| `0x0007C000 .. 0x0007FFFF` (`16 KiB`) | Active `JNL_A` (`LBA 874..889`) + read-only legacy `JNL_B` (`LBA 890..905`) | Monotonic `generation + CRC32 + COMMIT` + `FX/DX` LRU, warm-cached in `Ret_SRAM` |

### 1.2 5-Bank SRAM (`192 KiB`)

| Bank | Address Range | Resident Sections | Purpose |
|---|---|---|---|
| `SRAMH` | `0x1FFF8000 .. 0x1FFFFFFF` (`32 KiB`) | `.ram_vectors` (`1 KiB`), `.ramfunc` (`<= 9216 B` @ `0x1FFF8400`), `.dma_sramh`, `MSP` stack (`0x1FFFE000..0x1FFFFF00`) | Zero-wait ISR execution, EFM programming routines, DMA TX/RX buffers, `256 B` top guard |
| `SRAM1` | `0x20000000 .. 0x2000FFFF` (`64 KiB`) | `.data`, `.bss` | `adapter_ctx_t`, protocol state machines, CRC32 nibble table |
| `SRAM2` | `0x20010000 .. 0x2001FFFF` (`64 KiB`) | `.optics_sram2` | `optics_workspace_t`: double-buffered `CMD 0x24` A/B snapshots + `DS13` cache |
| `SRAM3` | `0x20020000 .. 0x20026FFF` (`28 KiB`) | `.diag_sram3` (`23 KiB` min) | `512 × 32 B` real-time diagnostic event ring + AF/parfocal flight trace rings |
| `Ret_SRAM` | `0x200F0000 .. 0x200F0FFF` (`4 KiB`) | `.retram_noload` | Warm-reset header (`BZWC`), `DS13` cache, `FX/DX` LRU cache, fault breadcrumbs (`BRYZ`) |

---

## 2. Source Layering & Hard Real-Time Constraints

- **`se-nz/src/00_hal/` (`-mgeneral-regs-only -fno-tree-vectorize -fno-builtin`)**: `192 MHz` MPLL clock startup & EFM cache setup (`sys_clock_sram.c`), `PA0` zero-backfeed UART shutdown pads (`gpio_pads.c`), `3 MHz` monotonic `TMRA1` (`tmra1_clock.c`), and dual-write protected `EFM` flash routines (`efm_ramfunc.c`).
- **`se-nz/src/01_transport/` (`-mgeneral-regs-only -fno-tree-vectorize -fno-builtin`)**: DMA-driven Z-mount `SPI3` slave (`z_spi3_slave.c`), `1 kHz` Z-mount `SPI1` hotline transmitter (`z_spi1_hotline.c`), E-mount `USART1` DMA bridge (`750 kbps` init → `1.5 Mbps` active with post-`0x0C` silent-link `750 kbps` fallback, `e_usart1_dma.c`), and `TMRA4` `60/120 Hz` VSync scheduler (`e_tmra4_vsync.c`).
- **`se-nz/src/02_core/` (Pure C17, Shared with Host ASan+UBSan Library)**:
  - `lens_session.c`: E-mount capability handshake (`HS_CAPS`), `ID 0x05`/`0x06` unpacking, `120 Hz` varifocal `30–60 Hz` cam-step false-MF suppression, `PA9` recovery/POR wake state machine, `reset_wake_parfocal_state()`.
  - `af_servo_bridge.c`: Active-AF (`0xE4`/`0xE6`) dynamic zoom target reprojection, post-zoom `zoom_guard_vsyncs` decay & `500 Hz` `SPI1` hotline velocity interpolation, `C0 -> E0` coast guard, `Body MF` active `0x1C` brake (`0x83`) with `b6_flags = 0x00` settling release, and opt-in `dist_code_x` closed-loop research parfocal (`af_servo_set_parfocal_research_mode`).
  - `aperture_controls.c`: Aperture lookup & ring translation, `1-LSB` kinematic + optical `0x117` distance fusion, slow-zoom frozen-distance reset guard (`reset_frozen_window_on_zoom()`).
  - `vr_duty_controller.c`: Split-axis IBIS/OIS ownership (`CMD 0xD2`..`0xD8`), `CMD 0x16` shutdown park state machine (`CMD 0x50/0x51`).
  - `optics_xlate_fpu.c` & `z_snapshot_builder.c`: On-the-fly Huffman/sparse `CMD 0x24` decompression, `81`-point spectral unpacking, live `ID 0x05` curve decoding, and double-buffered RAM-safe `SPI3` command snapshot publishing.
- **`se-nz/src/03_diag_storage/` & `se-nz/src/main.c`**: Wear-protected `32 B` Blackbox & `128 B` Journal storage, TX-busy atomic rollback around `lens_session_build_id04_wire()` in `service_vsync_id04()`, and main event loop.

---

## 3. Static Verification Gates (`se-nz/tools/`)

| Tool | Enforced Guarantees |
|---|---|
| `pack_pub.py` | Verifies `.PUB` container (`PRODUCTVER:1.001`), `.readme_ver` at `0x11000`, `MSP == 0x1FFFFF00`, `Reset_Handler` Thumb entry, `program_end <= 0x00060000`, zero secondary `PRODUCTVER:1.001` collisions on `512 B` sector boundaries, **0 VFP instructions** in `.ramfunc` / IRQ handlers / HAL / transport, and **0 Flash leaks** in RAM ISR & EFM call closures. |
| `check_source_comments.py` | Enforces clean technical vocabulary in all C/H/S/LD comments (`src/`, `include/`, `ld/`). |
| `arm-none-eabi-nm -u` | Confirms empty undefined symbol table (`0` `__aeabi_uldivmod` / `__aeabi_ldivmod` / `__udivmoddi4` helpers). |

---

## 4. Complete E→Z Reference Documentation / 深入实现文档

- Master Documentation Index: [`docs/README.md`](README.md)
- MCU Hardware & Firmware Design Specification: [`HC32F460_SE_NZ_DESIGN_SPEC.md`](ez-reference/HC32F460_SE_NZ_DESIGN_SPEC.md)
- Bootloader, Pin Roles & USB MSC Contract: [`BOOTLOADER_AND_USB_MSC_CONTRACT.md`](ez-reference/BOOTLOADER_AND_USB_MSC_CONTRACT.md)
- Z-Mount, E-Mount & PDAF Math Library: [`PART1_NIKKOR_Z_FLEET.md`](ez-reference/PART1_NIKKOR_Z_FLEET.md), [`PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md`](ez-reference/PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md), [`PART3_EZ_BRIDGE_AND_ERRATA.md`](ez-reference/PART3_EZ_BRIDGE_AND_ERRATA.md), [`PART4_PDAF_MATH_AND_CONTROL_OPT.md`](ez-reference/PART4_PDAF_MATH_AND_CONTROL_OPT.md)
- V2.24 Build Manifest, Field Verification & Known Limitations: [`SE_NZ_V224_VERIFICATION_AND_LIMITATIONS.md`](ez-reference/SE_NZ_V224_VERIFICATION_AND_LIMITATIONS.md)
- Ecosystem Catalog (`276` Lenses / `58` Spectral Curves / `92` 3D `CMD 0x24` Tables) & Historical V2.23 Archive: [`SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md`](ez-reference/SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md), [`SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md`](ez-reference/SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md)
