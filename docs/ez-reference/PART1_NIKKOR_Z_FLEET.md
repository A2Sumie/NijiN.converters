# Part 1 — Nikkor Z Native RXv2 Firmware Fleet Architecture Reference

**Scope & Verified Corpus**: This engineering reference documents the internal microcontroller topology, memory partitioning, clock tree, physical serial/DMA interfaces, inter-MCU bridge, actuator servo loops, trajectory generators, zoom/optical/teleconverter compensation tables, VR synchronization hooks, AF abort/soft-wall bitmask gates, and command dispatch tables across **24 unpacked Nikon Nikkor Z firmware packages** spanning **21 lens models** (including three firmware-revision pairs: `24-120mm f/4 S` `v1.10/v1.20`, `50mm f/1.8 S` `v1.10/v1.11`, and `800mm f/6.3 VR S` `v1.10/v1.11`).

Every instruction virtual address (VA), byte sequence, bitmask, fixed-point constant, timer reload value, and table dimension cited below is directly verified against the unpacked `.rx.bin` flash images under `work/banks/` and the container inventory in `fleet_container_census.json`.

---

## 1. Silicon Architecture, Memory Map, Startup CRT, and Clocks

### 1.1 Renesas RXv2 MCU Partitioning Across the Fleet

All 24 Nikkor Z packages target the **Renesas RXv2 (RX65N / RX651 family)** 32-bit little-endian architecture mapped at the top of the 4 GB physical address space (`0xFFE00000`–`0xFFFFFFFF`). Across the fleet, packages fall into two hardware configurations:

1. **Dual-MCU Lenses (`seg1` Main + `seg0` Sub-MCU)**:
   - **Body-facing Main MCU (`seg1`, 2 MB RX65N at `0xFFE00000`)**: Houses the Z-mount body SPI/DMA link (`SCI5`), body protocol state machine, 256-slot command dispatcher, optical/zoom/focus-breathing/thermal LUTs, EXRAM working state, and an asynchronous `0x55`-framed inter-MCU bridge (`SCI1` + `DMAC`) to `seg0`.
     - `1,835,008 B` (`1,792 KB`, `0xFFE00000..0xFFFC0000`): `70-200mm f/2.8 VR S` (`v1.21`), `400mm f/2.8 TC VR S` (`v1.10`), `600mm f/4 TC VR S` (`v1.10`).
     - `1,802,240 B` (`1,760 KB`, `0xFFE00000..0xFFFB8000`): `70-200mm f/2.8 VR S II` (`v1.10`), `100-400mm f/4.5-5.6 VR S` (`v1.10`), `180-600mm f/5.6-6.3 VR` (`v1.01`).
   - **Single-Package `0xFFE00000` with Unbundled Sub-MCU (`800mm f/6.3 VR S` `v1.10/v1.11`)**: Ships a single `1,802,240 B` (`0xFFE00000..0xFFFB8000`) main-MCU image labeled `seg0_bank0_ffe00000.rx.bin` by the container unpacker; its `SCI1` + `DMAC` `0x55` packet builder (`0xFFE15312..0xFFE15461`) is active, confirming an external sub-MCU absent from the public update file.
   - **Actuator Sub-MCUs (`seg0`, `0xFFF00000` or `0xFFF80000`)**:
     - `70-200mm f/2.8 VR S` (Gen I `seg0`): `1,007,616 B` (`984 KB`, `0xFFF00000..0xFFFF6000`), `INTB @ 0xFFF00C3C` (`42` active vectors), dual stepping-motor (`RSPI2` + `MTU`) and VR control loops.
     - `70-200mm f/2.8 VR S II` (Gen II `seg0`): `950,272 B` (`928 KB`, `0xFFF00000..0xFFFE8000`), `INTB @ 0xFFF00C78` (`31` active vectors), dual Hall-sensor ADC + `MTU3/4` dual-SSVCM servo control.
     - `400mm f/2.8 TC VR S`, `600mm f/4 TC VR S`, `100-400mm f/4.5-5.6 VR S`, `180-600mm f/5.6-6.3 VR` (`seg0`): compact `425,984 B` (`416 KB`, `0xFFF80000..0xFFFE8000`) sub-MCU images (`INTB @ 0xFFF80C78`, `21–26` active vectors).

2. **Single-MCU Lenses (`0xFFF00000` RX65N / RX651)**:
   - `851,968 B` (`832 KB`, `0xFFF00000..0xFFFD0000`): `50mm f/1.2 S`, `MC 105mm f/2.8 VR S`, `24-120mm f/4 S` (`v1.10/v1.20`), `24-70mm f/2.8 S` (`v1.20`), `24-50mm f/4-6.3`, `28-75mm f/2.8`, `DX 16-50mm f/3.5-6.3 VR`, `20mm f/1.8 S`, `24mm f/1.8 S`, `35mm f/1.8 S`, `50mm f/1.8 S` (`v1.10/v1.11`), `85mm f/1.8 S`.
   - `884,736 B` (`864 KB`, `0xFFF00000..0xFFFD8000`): `35mm f/1.4` (`v1.01`), `50mm f/1.4` (`v1.01`).
   - These images retain the shared Nikon SDK `SCI1` `0x55` framing helper in flash (e.g., `50mm f/1.2 S` at `0xFFF144A8`, `MC 105mm` at `0xFFF1776B`, `24-120mm` at `0xFFF8F08F`), with the `0x0008A023` `TDR1` reference isolated inside that library routine rather than wired to an active `SCI1` DMA/interrupt bridge.

### 1.2 Startup CRT (`ResetPRG`, `DTBL`, `BTBL`, `INTB`) & EXRAM vs SRAM0 Placement

Every image begins at its base VA with a 16-byte image header followed by the Renesas CC-RX C runtime bootstrap (`ResetPRG`), which writes `SYSTEM.PRCR` (`0x000803FE`) to unlock clock/wait registers, loads `INTB`, copies initialized data via `DTBL`, and zeroes `.bss` via `BTBL`.

The `DTBL` (`{u32 rom_src, u32 rom_end, u32 ram_dst}`) and `BTBL` (`{u32 ram_start, u32 ram_end}`) tables settle the exact RAM partitioning between `0xFFE00000` and `0xFFF00000` firmware classes:

| Image | Flash Base | `DTBL` Range (`+off`) | `.data` RAM Target | `BTBL` Main `.bss` Range | Shared Z-Mount Protocol Buffers (`BTBL`) | `INTB` Base (Active Vecs) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `70-200 G1 seg1` | `0xFFE00000` | `+0xC00..+0xC48` | **EXRAM `0x0081DF68`** (`0x8173E1..0x81DF68+`) | **EXRAM `0x0080A000..0x0081DF68`** (`71,593 B` `.bss`/`.data`) | **SRAM0** `0x3CC78..0x3F47A` (`10,242 B`) & `0x3F47C..0x3F5FE` (`386 B`) | `0xFFE00CA8` (`15`) |
| `70-200 G2 seg1` | `0xFFE00000` | `+0xC00..+0xC48` | **EXRAM `0x0081F5A4`** (`0x81C77E..0x81F5A4+`) | **EXRAM `0x0080A000..0x0081F5A4`** | **SRAM0** `0x3CC78..0x3F47A` (`10,242 B`) & `0x3F47C..0x3F5FE` (`386 B`) | `0xFFE00CA8` (`19`) |
| `400TC seg1` | `0xFFE00000` | `+0xC00..+0xC48` | **EXRAM `0x0080EE38`** (`0x80D32C..0x80EE38+`) | **EXRAM `0x0080A000..0x0080EE38`** | **SRAM0** `0x3CC78..0x3F47A` (`10,242 B`) & `0x3F47C..0x3F5FE` (`386 B`) | `0xFFE00CA8` (`20`) |
| `50mm f/1.2 S` | `0xFFF00000` | `+0xC00..+0xC48` | **SRAM0 `0x0001B7EC`** (`0x1A34E..0x1BCE8`) | **SRAM0 `0x00018C00..0x0001B7EC`** | **SRAM0** `0x3CC78..0x3F47A` (`10,242 B`) & `0x3F47C..0x3F5FE` (`386 B`) | `0xFFF00CA8` (`13`) |
| `MC 105mm VR S` | `0xFFF00000` | `+0xC00..+0xC48` | **SRAM0 `0x0001CA70`** (`0x1AA72..0x1D010`) | **SRAM0 `0x0001AFA4..0x0001CA70`** | **SRAM0** `0x3CC78..0x3F47A` (`10,242 B`) & `0x3F47C..0x3F5FE` (`386 B`) | `0xFFF00CA8` (`17`) |
| `70-200 G1 seg0` | `0xFFF00000` | `+0xC00..+0xC24` | **SRAM0 `0x0000EAF8`** (`0xE09B..0xEBE4`) | **SRAM0 `0x0000C000..0x0000EAF8`** | N/A (sub-MCU) | `0xFFF00C3C` (`42`) |
| `70-200 G2 seg0` | `0xFFF00000` | `+0xC00..+0xC48` | **SRAM0 `0x0001AE58`** (`0x197EE..0x1AF90`) | **SRAM0 `0x00018400..0x0001AE58`** | N/A (sub-MCU) | `0xFFF00C78` (`31`) |
| `400TC seg0` | `0xFFF80000` | `+0xC00..+0xC48` | **SRAM0 `0x0001A274`** (`0x196EB..0x1A3DC`) | **SRAM0 `0x00018400..0x0001A274`** | N/A (sub-MCU) | `0xFFF80C78` (`26`) |

Key architectural consequences of this layout:
- **RX65N Extended SRAM (`EXRAM`)**: Addresses `0x00800000..0x0087FFFF` on `2 MB` RX65N parts (`0xFFE00000` `seg1` images and `800mm f/6.3`) hold the main application `.bss`/`.data` (`0x0080A000..`), optical pipelines, and double-buffered telemetry slabs (`0x814E52` on Gen I; `0x81AC50`, `0x81AE51`, `0x81F810` on Gen II).
- **Fixed Z-Mount Protocol Region (`SRAM0`)**: Across every single `seg1` and single-MCU image, the body MOSI receive ring (`0x0003CC78..0x0003F47A`, `10,242 B`) and MISO reply buffer (`0x0003F47C..0x0003F5FE`, `386 B`), along with internal state page `0x000A4000..` (`0xA403F`, `0xA4044` lens slot ID, `0xA4057`, `0xA4062`), occupy identical SRAM addresses regardless of EXRAM usage.
- **Data Flash Calibration Seed (`0x00100000` -> Low SRAM)**: Main/single-MCU firmware images read factory calibration blocks from RX65N Data Flash (`0x00100000`, verified in `50/1.2` at `0xFFF0F5C6/0xFFF0FAB7`, `400TC` at `0xFFE044F4`, `70-200 G2` at `0xFFE064F0`), populating low SRAM calibration structs at `0x0480` (`70-200 G2` `0xFFE7D353`, `0xFFF1490C`), `0x0890` (`70-200 G2` `0xFFF1BC42`), `0x0C10` (`400TC` thermal breakpoint table at `0xFFE368F3`), `0x6C10` (`50/1.2` & `MC105` optical identity parameters), and `0x8C10` (`400TC` & `70-200 G2` optical identity parameters).

### 1.3 System Clocks & Timer Base Rates

Clock initialization across main and sub-MCUs programs an identical RX65N PLL configuration from an `8 MHz` crystal (`XTAL`):
- `SYSTEM.SCKCR` (`0x00080020`) = `0x10C60111` (`FCK = /2`, `ICK = /1`, `PCLKA = /1`, `PCLKB/C/D = /2`).
- `SYSTEM.SCKCR2` (`0x00080024`) = `0x0011`, `SYSTEM.PLLCR` (`0x00080028`) = `0x1D00` (`×15` multiplier -> `120 MHz` PLL output), `SYSTEM.MEMWAIT` (`0x0008101C`) = `2` (`2` flash wait states).
- Resulting domain frequencies: **`ICLK = 120 MHz`**, **`PCLKA = 120 MHz`**, **`PCLKB = PCLKC = PCLKD = 60 MHz`**, **`CMT` `PCLK/8` tick = `7.5 MHz` (`133.33 ns`)**.
- Verified CMT period registers:
  - `70-200 G2 seg1`: `CMT1.CMCOR = 0x1D4C` (`7,500` ticks @ `7.5 MHz`) = exact **`1.000 ms` (`1 kHz`)** system tick.
  - `70-200 G1 seg0` & `50mm f/1.2 S` (`0xFFF212C6`): `CMT3.CMCOR = 0x0753` (`1,875` ticks @ `7.5 MHz`, `CMCR = 0x00C0`) = exact **`250.0 µs` (`4.000 kHz`)** actuator servo tick, serviced alongside Independent Watchdog Timer (`IWDT 0x00088020`, refreshed with `0x00`/`0xFF`).

---

## 2. Wire Protocols, SPI Dispatch Tables (`24B` vs `28B`), and Inter-MCU Bridge

### 2.1 Body <-> Lens Z-Mount Interface (`SCI5` + `DMAC`)

All body Z-mount communication on `seg1` / single-MCU images runs over synchronous serial **`SCI5` (`0x0008A0A0`, `TDR5 = 0x8A0A3`, `RDR5 = 0x8A0A5`)** paired with **`DMAC` (`0x000820xx`, interrupt `vec[120]`)**:
- **Frame header layout in MOSI (`0x3CC78`, `mosi[i] == *(0x3CC78 + i)`)**:
  - `[0] = 0x55` sync byte
  - `[1]`: sequence / transport byte
  - `[2]`: lens target slot ID (checked by `seg1` against RAM `[0x000A4044]`)
  - `[3]`: command opcode `cmd` (`0x00..0xFF`)
  - `[4]`: subcommand ID `sub`
  - `[5..6]`: 16-bit little-endian payload length (`<= 0x00C0` on standard command frames, up to `0x1400` on bulk blocks)
  - `[7]`: additive negative header checksum (`0x00 - sum([0..6]) & 0xFF`)
- **Dispatch loop (`70-200 G1 @ 0xFFF55402` -> `0xFFECD770`; `70-200 G2 @ 0xFFEF6B4D` -> `0xFFEEBEE4`)**:
  Verifies `0x55`, computes additive header checksum, and indexes a `256 × 12-byte` top-level table at `{u32 sub_ptr, u32 sub_cnt (low byte), u32 enable_flag}`.

### 2.2 Subcommand Descriptor Stride: `24 B` (Standard) vs `28 B` (`70-200mm f/2.8 VR S II` Unique)

Each enabled entry in the `256 × 12 B` table points to an array of `sub_cnt` subcommand descriptors scanned linearly by `sub_id`:
- **23 of 24 packages (`24-byte` descriptors = `6 × uint32`)**:
  `{ +0x00: sub_id, +0x04: rx_fn, +0x08: tx_fn, +0x0C: dir, +0x10: tx_flag, +0x14: data_ptr }`
- **`70-200mm f/2.8 VR S II` (`28-byte` descriptors = `7 × uint32`, `mul #0x1c` in `0xFFEF6B4D`)**:
  Appends **`+0x18: max_payload_bytes`**, acting as a per-subcommand DMA/payload length ceiling. On `CMD 0x24`, subcommands `{0x00, 0x01, 0x02}` carry `+0x18 = 0x1E00` (`7,680 B`, pointing to EXRAM descriptors `0x81F810` / `0x81F818`), while `{0x10, 0x11, 0x12}` carry `+0x18 = 0x0D8A` (`3,466 B`, pointing to `0x81F814` / `0x81F81C`).

Across the 24 packages, populated command counts (`sub_ptr != 0`) partition cleanly into four profiles:
1. **Standard 65-slot profile (20 packages)**: `63` active (`enable_flag = 1`) + `2` disabled (`0x70`, `0x71` with non-null `sub_ptr` and `enable_flag = 0`).
2. **66-slot profile (`180-600mm f/5.6-6.3 VR`, `35mm f/1.4`, `50mm f/1.4`)**: adds **`CMD 0xB0`** (`enable_flag = 1`, `cnt = 1`), writing a 38+-byte MOSI payload into an application mailbox (`0x0080A11A` base on `180-600` via `0xFFEF0027`; `0x00018D1A` on `35/1.4` via `0xFFF110EF`).
3. **67-slot profile (`70-200mm f/2.8 VR S II`)**: adds both **`CMD 0xB0`** (`0xFFEF9798`) and **`CMD 0xB1`** (`0xFFEF9803`) (`28 B` descriptors), which copy identically formatted 38+-byte frames from `0x3CC79` / `0x80CB88` into EXRAM mailbox `0x80C91C` (`0x3CC79 + 0x7CFCA3`).
4. **62-slot entry profile (`DX 16-50mm f/3.5-6.3 VR`)**: omits `0x30`, `0x31`, and `0xEB` (`60` active + disabled `0x70`, `0x71`).

### 2.3 Key Z-Mount SPI Command Semantic Reference

| Command | Sub(s) | Direction / Role | Verified ROM Implementation Evidence |
| :--- | :--- | :--- | :--- |
| `0x10` | `0x00` | Identity & capability query | `rx`: stores 5 header bytes to `0xA403F`; `tx` (`50/1.2 @ 0xFFF129AA`, `400TC @ 0xFFE20C0A`, `G1 @ 0xFFF56EC4`): packs capability/identity bytes into `0x3F47D` (`miso[1..]`) from RAM (`0x19255`), `*ptrs` (`*0x1BA10` / `*0x1CDA4` / `*0x81E524`), common page `[0xA4057]`, and Data-Flash-seeded struct `0x6C10`/`0x8C10` offsets `+0x22..+0x25`. |
| `0x12` | `0x00` | Optical & mechanical descriptor | `tx` (`50/1.2 @ 0xFFF12B14`, `G1 @ 0xFFF57021`): builds 49-byte `0x3F47C` (`miso[1..0x30]`) reply from `*0x1BA10`/`*0x81E524` (`+0x00..+0x23`, `+0x31..+0x32`) plus `0x198A1`, `0x18F79`; `G1` sets `miso[0x24..0x27] = 0x03000000` (`0xFFF570B3`) and `miso[0x28] = 0x08` (`0xFFF570BA`). |
| `0x20` | `0x00`, `0x01` | Live lens/optical telemetry | `sub 0` (`G1 @ 0xFFF57271`, `G2 @ 0xFFEF8F66`, `400TC @ 0xFFE20FD2`, `MC105 @ 0xFFF1603C`): reads double-buffered `256 B` block (`0x814E52` on `G1`, `0x81AC50` on `G2`, `0x80B9B1` on `400TC`, `0x19267` on `50/1.2`/`MC105`) with `bnot #0` page toggle (`256 B` copy via `smovf`) plus `16 B` live trailer (`+0x4E4` -> `+0x145..+0x148` or `+0x506` -> `+0x536` on `G2`). `sub 1` copies `22 B` (`mul #0x16`) from double-buffered focus/position table. |
| `0x22` | `0x13`, `0x20`, `0xA0` | Synchro / VR & extended state | `sub 0x13` & `0xA0`: push body commands into EXRAM/SRAM mailbox (`0x80C836` on `G2`); `sub 0x20`: bidirectional (`G2` `0xFFEF92DD` stores `60 B` (`mul #0x3C`) body sync frames in `0x81B0DA`; `0xFFEF934D` returns `256 B` double-buffered block from `0x81AE51` via `bnot #0` + `smovf`). |
| `0x24` | `0x00..0x02`, `0x10..0x12` | Static calibration/data descriptor | Null `rx`/`tx` with `dir = 0`, exposing ROM/RAM table descriptors (`50/1.2`: `0x1B9D8/0x1B9DC`; `MC105`: `0x1CD6C/0x1CD70`; `G1`: `0x81E518/0x81E51C`; `G2`: `0x81F810/0x81F818` capped at `0x1E00`, `0x81F814/0x81F81C` capped at `0x0D8A`). |
| `0xA0..0xA3` | `0x00..` | Lens function / control ring / button state | `50/1.2` (`0xFFF131FA` / `0xFFF13275`): accepts host ring/control config and reads `22 B` (`mul #0x16`) double-buffered switch/ring telemetry from `0x18E0A` (`0x18F6A` index). |
| `0xB0`, `0xB1` | `0x00` | Extended body push (`G2`/`180-600`/`35f1.4`/`50f1.4`) | Copies 38+ bytes from body SPI packet into mailbox (`0x80C91C` on `G2`). |
| `0xC0`, `0xE2`, `0xE4`, `0xE6`, `0xE8` | multiple | AF drive / move / brake commands | See **§5** for verified per-command abort bitmasks (`and #6; cmp #6` vs `tst #6`) and soft-wall clamps. |
| `0xD2..0xD5`, `0xD7..0xD8` | `0x00` | VR configuration & status | `G2` (`0xFFEF98D9..0xFFEF9B13`), `G1` (`0xFFF57A6E..0xFFF57C99`), `MC105` (`0xFFF16897`), `400TC` (`0xFFE217FD`), `50/1.2` (`0xFFF135D4`): present uniformly in dispatch tables, queuing 32+-byte body payloads into VR/control mailbox (`0x80C94A` on `G2`, processed at `0xFFF2CDDE`). |
| `0xF3` | `0x00` | Status / diagnostic telemetry | `50/1.2` (`0xFFF140C6`): copies 6 live status bytes from `0x18F99` into `0x3F47C + 0x1..0x6`. |

### 2.4 Main <-> Sub (`seg1` <-> `seg0`) Inter-MCU Bridge (`0x55` Protocol)

Dual-MCU lenses bridge `seg1` (`0xFFE00000`) to `seg0` (`0xFFF00000` / `0xFFF80000`) over dedicated high-speed asynchronous/synchronous UART+DMAC:
- **`seg1` Link Hardware**: **`SCI1` (`0x0008A020`, `BRR1 = 0x8A021` programmed `0x05`/`0x02`/`0x01`, `TDR1 = 0x8A023`) + `DMAC` (`0x00082200`, `vec[122]`)**.
  - Packet builder (`70-200 G1 @ 0xFFF5892A`, `70-200 G2 @ 0xFFEFA9C7`, `800/6.3 @ 0xFFE15312`): `[0] = 0x55, [1] = seq, payload..., [-sum & 0xFF]`.
- **`seg0` Link Hardware & Dispatch Architecture**:
  - **`70-200mm G1 seg0`**: `SCI1` link; `256 × 12 B` `0x55` dispatch table at **`0xFFF52948`** with **`22` active commands**: `0x00, 0x10, 0x12, 0x13, 0x31, 0x50, 0xD2, 0xD3, 0xD4, 0xD5, 0xD7, 0xD8, 0xE0, 0xF0, 0xF1, 0xF2, 0xF3, 0xF5, 0xF7, 0xF8, 0xFA`, plus `0xFE` (`12` subcommands).
  - **`70-200mm G2 seg0`**: `SCI1` (`0x8A022`) + `DMAC` (`0x820C0`/`0x820D3`); `256 × 12 B` `0x55` dispatch table at **`0xFFF02A84`** with **`10` active commands**: `0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF7, 0xF8, 0xFA`, plus `0xFE` (`0xFFF028F4`, **`10` subcommands with `20 B` stride** `{u32 sub_id, rx_fn, tx_fn, dir, tx_flag}`: IDs `[0x00, 0x8F, 0xF0, 0xF2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8]`). Notice that G2 offloads optical/VR protocol parsing (`0x10..0xE0`) onto `seg1` EXRAM, leaving `seg0` strictly focused on actuator commands (`0xF0..0xFE`).
  - **`400TC / 600TC / 100-400 / 180-600 seg0` (`0xFFF80000`)**: Replaces the `256 × 12 B` table with a compact two-path `0x55` handler wired on **`seg0` `SCI5` (`0x8A0A2/0x8A0A3`) + `DMAC` (`0x82100/0x82140`)**:
    1. Long/framed `0x55` validator (`400TC @ 0xFFF86DB6`, `600TC @ 0xFFF86DC6`, `100-400 @ 0xFFF86201`) checking `0x55` + 16-bit checksum before indirect `jsr [0x531[r1]+4]` and replying via DMA with `0x54` (`0x87414`, `0xFFF86EA1`).
    2. Fixed `22 B` (`0x16`) `0x55` frame validator (`400TC @ 0xFFF87632`, `600TC @ 0xFFF87642`, `100-400 @ 0xFFF86A80`) verifying byte-wise negative sum and handling `0xF0` (`0xFFF87664`) or generic `0x55` telemetry (`0xFFF87615`).

---

## 3. Deep Comparative Engineering: `70-200mm f/2.8 VR S` (Gen I) vs `70-200mm f/2.8 VR S II` (Gen II) & `400TC/600TC`

### 3.1 Actuator & Control Loop Architecture (`seg0` Comparison Matrix)

| Subsystem | `70-200mm f/2.8 VR S` (`Gen I, v1.21`) | `70-200mm f/2.8 VR S II` (`Gen II, v1.10`) | `400mm f/2.8 TC` / `600mm f/4 TC` (`v1.10`) |
| :--- | :--- | :--- | :--- |
| **Actuator Hardware** | Dual Stepping Motors (**Dual-STM**) + encoder feedback | Dual Linear Voice-Coil (**Dual-SSVCM**) + dual Hall sensors | Dual-SSVCM super-telephoto linear voice-coil |
| **`seg0` Flash / `INTB`** | `1,007,616 B @ 0xFFF00000`, `INTB @ 0xFFF00C3C` (`42` vecs) | `950,272 B @ 0xFFF00000`, `INTB @ 0xFFF00C78` (`31` vecs) | `425,984 B @ 0xFFF80000`, `INTB @ 0xFFF80C78` (`26` vecs) |
| **Startup Poll Port** | `0x0008C041` bit 2 | `0x0008C04D` bit 0 | `0x0008C04D` bit 5 |
| **Main Loop / WDT** | Refreshes `IWDT 0x88020` (`0x00/0xFF`), `CMTW0 0x94280` | Main dispatch via `0xFFF4D2E0`, refreshes `IWDT` (`CMT1` / `CMTW`) | `CMT0.CMCR = 0x81` (`PCLK/32`), `CMT2.CMCR = 0x80` (`PCLK/8`) |
| **Driver / ADC ISRs** | `RSPI2 0xD0304`: `INTB[109]` wrapper `0xFFF65F91` -> `0xFFF556FE` (24-bit SPI); encoder `INTB[68/69]` wrappers `0xFFF65CB5`/`0xFFF65CEF` -> `0xFFF652C4`/`0xFFF65363` timed against `CMT1 0x88014` | Dual ADC (`vec[68,69,70,75]`) writing Hall feedback channels `ch0 @ 0x19890` & `ch1 @ 0x198BC` | Hall ADC + `MTU3` `0xC1200`/`0xC1A00`, `0xC130C`/`0xC130E` |
| **PWM / Timer Carrier** | `CMT3 = 0x0753` (`1,875` ticks -> `4 kHz / 250 µs`); `MTU` stepping gates (`0xFFF53800`) | `MTU` `0xC1200`/`0xC1A00`, `TGRA/B = 0x6E27 / 0x752F` (`28,200 / 30,000`), PWM writer `0xFFF53605` | `MTU` `0xC1200`/`0xC1A00`, `TGRA = 0x752F` (`29,999`) |
| **Trajectory Law** | 256-byte ramp generator (`0xFFF55B87`) + `14 B` motor descriptors (`0xFFF497D0`) | **5-state (`0..4`) piecewise velocity/accel-clamped generator (`0xFFF52266..0xFFF52F31`)** | **6-segment C² Quintic minimum-jerk polynomial solver (`400TC @ 0xFFF8F2B6`, `600TC @ 0xFFF8F3EF`)** |
| **Cam / Floating Tables** | **4 × `72,669 B` (`0x11BDD`) blocks** (`0xFFF0281D`, `0xFFF143FA`, `0xFFF25FD7`, `0xFFF37BBC`) selected by `[0xE137]` (`0xFFF64654`) | Dual-axis floating focus coupling (`0xFFF548B0`/`0xFFF55010`) + `seg0` LUTs (`0xFFF03684..0xFFF4891C`, `0xFFF47A1C`) | `16`-entry breakpoint table (`0xFFF8F1EB` -> `0xFFF84870`), `6 × 72 B` segment buffer (`0x19ECC` [`0x19EC8+4`] on `400TC`; `0x19ED4` [`0x19ED0+4`] on `600TC`) |
| **PID / Control Structs** | Dual-STM phase/step tracking (`0xFFF55A70`, `0xFFF56200/0xFFF5627B`) | Dual-channel PID (`0xFFF568A8` invoked via 16-entry dispatch table `0xFFF4981C` at `0xFFF5486D`, slot `0xFFF49844`) | Dual-channel PID (`0xFFF8FF1D..0xFFF903B8`) writing clamped duty to `MTU3 0xC130C`/`0xC130E` |

### 3.2 Control Law Comparison: Gen I STM vs Gen II V/A-Clamped SSVCM vs 400TC/600TC Quintic Spline

1. **`70-200mm G1` Dual-STM Driver Pipeline (`seg0`)**:
   - Four contiguous `72,669 B` (`0x11BDD`) calibration/cam/ramp blocks sit at `0xFFF0281D`, `0xFFF143FA`, `0xFFF25FD7`, and `0xFFF37BBC` (`blk2 @ 0xFFF25FD7` omits the `"NIKON"` ASCII tag present at `+0x318` in `blk0/blk1/blk3`). Selector `0xFFF64654` picks the active block from SRAM state byte `[0x0000E137]`.
   - Stepper driver `0xFFF5627B` indexes a `14 B` motor-descriptor table (`0xFFF497D0`), while `0xFFF55B87` builds `256 B` velocity ramp lookup profiles. At runtime, `CMT3` (`250 µs` = `4 kHz`) coordinates step updates, `INTB[109]` (`0xFFF65F91` -> `0xFFF556FE`) shifts 24-bit SPI words over `RSPI2` (`0xD0304`), and encoder edges trigger `INTB[68/69]` (`0xFFF65CB5`/`0xFFF65CEF` -> `0xFFF652C4`/`0xFFF65363`) reading `CMT1.CMCNT` (`0x88014`) for closed-loop step verification.

2. **`70-200mm G2` Dual-SSVCM 5-State V/A-Clamped Trajectory & Dual-Axis PID (`seg0`)**:
   - G2 replaces stepper LUTs with closed-loop Hall-effect position control for two independent voice-coil focus groups (`ch0` state at `0x19890`, `ch1` at `0x198BC`, fed by `ADC` `vec[68,69,70,75]`).
   - Function `0xFFF52266` (`24` direct callers across `0xFFF55FB8..0xFFF5BA00`) runs a **5-state (`0..4`) piecewise velocity/acceleration-clamped profile** (verified by `emulu`/`mul` strides `0x1C`, `0x88`, `0xD0` and fixed-point reciprocal divisors `0x10624DD3` [`/1000`], `0x66666667` [`/10`], `0x51EB851F` [`/100`]), coupled through floating compensation helpers (`0xFFF548B0` -> `0xFFF55010`). It contains no quintic spline solver.
   - Task dispatcher `0xFFF5486D` indexes the 16-entry control function table at `0xFFF4981C` (`0xFFF49844 -> 0xFFF568A8`), which runs the dual-channel PID (`ch=0,1`) and updates `MTU3/4` PWM duty via `0xFFF53605` (`0xC1200`/`0xC1A00`, `TGRA/B = 28,200/30,000`).

3. **`400mm TC` & `600mm TC` C² Quintic Minimum-Jerk Trajectory Solver (`seg0`, Contrast vs `70-200 G2`)**:
   - Unlike `70-200 G2`'s 5-state velocity/accel clamp, `400TC seg0` (`0xFFF8D742` 8-state jump table `0xFFF911F6`) and `600TC seg0` implement a **true 6-segment, 64-bit Q13/Q14 C² quintic Hermite spline solver** (`400TC @ 0xFFF8F2B6`, `600TC @ 0xFFF8F3EF`):
     - `0xFFF8F1EB` maps target travel via a 16-entry breakpoint table (`0xFFF84870`).
     - `0xFFF8EFD5` (`400TC`) / `0xFFF8F10E` (`600TC`) zeroes a `6 × 0x48` (`72 B`) segment descriptor buffer (`432 B` at `0x19ECC` = `0x19EC8 + 4` in `400TC`; `0x19ED4` = `0x19ED0 + 4` in `600TC`).
     - `0xFFF8F2B6` (`400TC`) / `0xFFF8F3EF` (`600TC`) solves boundary conditions $(p, v, a)$ using exact integer Hermite coefficients **`{20 (0x14), 12 (0x0C), 6 (0x06), 30 (0x1E), 14 (0x0E), 3 (0x03)}`** in 64-bit fixed-point (`0xFFF90B70` 64-bit divider, `/250`, `/1000`, `/2000`), advancing segments via `0xFFF8FC24`.
     - PID controller `0xFFF8FF1D..0xFFF903B8` outputs clamped duty cycles to `MTU3 0xC130C` / `0xC130E` using fixed-point scaling (`0x66666667`, `0x51EB851F`).

### 3.3 Zoom-Dependent Optical/Cam Tables (`K_Z`), Floating Compensation, & VR Sync (`seg1`)

Both `70-200 G1 seg1` and `70-200 G2 seg1` keep optical geometry, focus-sensitivity (`K_Z`), breathing/focal LUTs, and VR command queues on `seg1`:
- **`70-200 G1 seg1`**:
  - Inside the `CMD 0xC0` optical/drive lookup (`0xFFF70E9E`), zoom index `0x182C[r6]` and focus/cam indices address a **`0xF3C0` (`62,400 B` = `480 × 130 B`) per-zoom-bin table at `0xFFE977C8`** (`mul #0x82, r3` = `130 B` row stride; `mul #0xF3C0, r2` = zoom bin stride at `0xFFF70EBB..0xFFF70EC1`), selecting `[r9]` vs `[r9+4]` via `btst #0, r12` + `0xFC 0x4B 0x92` before computing optical bounds.
  - `CMD 0x20 sub 0` reads double-buffered EXRAM telemetry block `0x814E52` (`0xFFF57271`).
  - `CMD 0x22` (`0x13`, `0x20`, `0xA0`) and `CMD 0xD2..0xD8` (`0xFFF57A6E..0xFFF57C99`) handle VR / Synchro VR state sync.
- **`70-200 G2 seg1`**:
  - EXRAM optical telemetry uses **two** distinct double-buffered `256 B` blocks: `0x81AC50` (`CMD 0x20 sub 0 @ 0xFFEF8F66`) and `0x81AE51` (`CMD 0x22 sub 0x20 @ 0xFFEF934D`), paired with a **`60-byte` (`mul #0x3C`) body Synchro/VR input ring at `0x81B0DA`** (`0xFFEF92DD`).
  - Optical evaluation pipeline `0xFFEFC330` invokes `0xFFEFBE8A`, `0xFFEFBF6A`, `0xFFEFC0D1` (VR/optical flag bitmasking at `0xA401C`), `0xFFEFC635`, `0xFFEFC653`, `0xFFEFC6D0` (writing `0x81C4DA`-rooted telemetry structures into `0x81F5B0`/`0x81F5C0`), `0xFFEFDA63` (writing `0x81B2EA` telemetry via a **3D LUT of `62 × 2 B` (`0x7C = 124 B`) × `20` sub-bins (`0x9B0 = 2,480 B`) per zoom bin** at `*0x81F838`, `0xFFEFDB5D..0xFFEFDB8B`), and `0xFFF1BC13`.
  - Focus/zoom sensitivity and pupil/cam interpolation (`0xFFF0E1FA`, `0xFFF0FB68`) address **`0xFFEB6248`** with `mul #0x82` (`130 B` = `65 × uint16` rows per zoom step, e.g., column 0 descending `0x0C90` (`3216`) -> `0x044E` (`1102`) and column 1 `0x0C6C` (`3180`) -> `0x04D8` (`1240`)), followed by `mul`/`div` scaling (`0xFFF0E20B`/`0xFFF0E20D`).
  - Startup fallback `0xFFF14904` reads factory calibration flags from SRAM `0x0480` or falls back to `*0x81F810` (`CMD 0x24` EEPROM/NVRAM mirror).

---

## 4. Spotlight Technical Profiles

### 4.1 `Nikkor Z 50mm f/1.2 S` (`v1.10`)
- **MCU & Memory**: Single RX65N (`851,968 B @ 0xFFF00000..0xFFFD0000`, `model_id = 0x15`, `INTB @ 0xFFF00CA8`, `13` active vectors). `.data` runs in SRAM0 `0x0001B7EC..0x0001BCE8`, `.bss` in `0x00018C00..0x0001B7EC`, Z-mount protocol buffers in `0x3CC78..0x3F5FE`.
- **Servo & Watchdog**: Uses the `4.000 kHz` (`250 µs`) `CMT3.CMCOR = 0x0753` (`1,875`) tick (`0xFFF212C6`) and `IWDT 0x88020` (`0xFFF212E4`) directly in the single MCU alongside body SPI handling (`0xFFF12987..`).
- **Optical Identity Struct**: `.data` pointer `0x0001BA10` initializes to ROM `0xFFF0B3DA` (first 64 B: `15 00 00 00 00 00 00 01 03 01 98 04 00 00 00 40 ... 22 4B 33 00 22 4B 33 00 3C 00 00 01 44 00 00 00 ... "NIKKOR Z 50mm f/1.2 S"`), encoding lens ID `0x15`, VR capability byte `+0x02 = 0x00` (no optical VR), optical focus-scale word `0x00334B22` (`+0x1C` / `+0x20`), and identity flags read by `CMD 0x10` (`0xFFF129AA`) and `CMD 0x12` (`0xFFF12B14`). Even without optical VR, `CMD 0xD2` (`0xFFF135D4`) remains active (`32+` B mailbox write to `0x18D48`).

### 4.2 `Nikkor Z 400mm f/2.8 TC VR S` (`v1.10`) & `Nikkor Z 600mm f/4 TC VR S` (`v1.10`)
- **MCU & Control Split**: Both pair a `1,835,008 B` (`0xFFE00000`, EXRAM `0x0080A000..`) `seg1` main MCU (`model_id = 0x1E` on 400TC, `0x25` on 600TC) with the shared `425,984 B` (`0xFFF80000`) SSVCM quintic sub-MCU (`seg0`), linked via `seg1` `SCI1` <-> `seg0` `SCI5` (`0x8A0A2/0x8A0A3`) using the two-path `0x54`/`0x55` protocol.
- **Built-in 1.4x Teleconverter (`TC-IN` vs `TC-OUT`) & Thermal Compensation (`400TC seg1` & `600TC seg1`)**:
  - **Table switching by TC mechanical state (`400TC @ 0xFFE28750..0xFFE28820` & `0xFFE2BFE0..0xFFE2C0D0`)**: Inspects the teleconverter state flag (`0x23E[r6]` / `[r6]`), selecting ROM boundary table **`0xFFE1AB1A`** when `TC-OUT` (`400mm f/2.8`) and **`0xFFE1A518`** when `TC-IN` (`560mm f/4.0`), mapped against common table `0xFFE1A216` with `+0`/`+4` zone-boundary hysteresis and updating telemetry status byte `0x80BEE8`. Focus-breathing effective-focal-length LUT `0xFFE1A400` stores `540..563 mm` (`TC-IN`) values (`600TC` carries parallel `(600, 840)` / `(40, 56)` descriptor pairs at `0xFFE09F61` and `0xFFE0E5BB`).
  - **Thermistor-compensated TC focus limits (`400TC @ 0xFFE32DE5..0xFFE32E82` -> `0xFFE368EB`)**: Tests TC sensor bit `btst #1, 0x2097[r6]`; on state or zone change (`[0x169B + r6]`), shifts zone index by `shll #3` (`8 B` entry stride) into **`0xFFE1D2B8`** (pairs of 32-bit values `[native, TC]`, e.g. `0x0002C319, 0x0002C319, 500, 530, 561, ...`) and calls `0xFFE368EB`, which reads thermistor byte `[0x0080BF62]` against a 4-point piecewise temperature calibration table at SRAM `0x0C10` (`+0x2E..+0x31` temperature breakpoints, `+0x32..+0x39` drift offsets).

### 4.3 `Nikkor Z MC 105mm f/2.8 VR S` (`v1.10`)
- **MCU & Memory**: Single RX65N (`851,968 B @ 0xFFF00000`, `model_id = 0x18`, `INTB @ 0xFFF00CA8`, `17` active vectors, `.bss` `0x0001AFA4..0x0001CA70`, `.data` `0x0001CA70..0x0001D010`).
- **Optical & VR Descriptor (`*0x1CDA4 -> 0xFFF101C1`)**: First 64 B (`18 00 04 00 00 00 00 01 03 01 9C 04 03 00 00 3E ... 43 DC 66 00 43 DC 66 00 ... 19 00 FE FF FE FF "NIKKOR Z MC 105mm f/2.8 VR S"`). Contrasting directly with `50mm f/1.2 S` (`0xFFF0B3DA`), `MC 105` encodes `+0x02 = 0x04` (VR hardware present, serviced by `CMD 0xD2 @ 0xFFF16897`), `+0x0C = 0x03`, optical scale `0x0066DC43` (`~2.005×` the `0x00334B22` word on `50/1.2`), and `+0x31 = 0x19` (`25`), copied by `0x10_tx` (`0xFFF15C6D`), `0x12_tx` (`0xFFF15D02`), and `0x20_tx` (`0xFFF1603C`).

### 4.4 `Nikkor Z 24-120mm f/4 S` (`v1.10/v1.20`) vs `Nikkor Z 100-400mm f/4.5-5.6 VR S` (`v1.10`)
- **`24-120mm f/4 S` (`model_id = 0x1F`)**: Single-MCU `851,968 B @ 0xFFF00000`, `11` active `INTB` vectors, no EXRAM (`.bss` `0x1A270..0x1B314` in `v1.10` / `0x1A278..0x1B31C` in `v1.20`). Both `v1.10` and `v1.20` keep identical SPI table base `0xFFF5D188` (`65` slots, `24 B` stride) and `0x10`/`0x22`/`0xD2` handler addresses (`0xFFF8D56E`/`0xFFF8DC6B`/`0xFFF8E1BB`).
- **`100-400mm f/4.5-5.6 VR S` (`model_id = 0x1C`)**: Dual-MCU (`1,802,240 B seg1 @ 0xFFE00000` with EXRAM `0x0080A000..0x0081D050+`, SPI table `@ 0xFFED9CC0` `65` slots/`24 B` stride, plus `425,984 B seg0 @ 0xFFF80000` sharing the two-path `SCI5` `0x55` sub-MCU link `0xFFF86201`/`0xFFF86A80` with `400TC/600TC/180-600`).

---

## 5. Mandatory Machine-Code Proof: AF Brake/Abort Bitmasks & Soft-Wall Margins

### 5.1 `Nikkor Z 50mm f/1.2 S` (`seg0_bank0_fff00000.rx.bin`) Bit-Exact Verification

In `50mm f/1.2 S`, the AF drive/move command handler is entered via jump table `0xFFF3A742` (`0xFFF271E7..0xFFF27212`, indexed by `cmd - 0xC0`), where `r6 = 0x00018C00` and `r6 + 5` holds `mosi[0]` (`0x6[r6] == mosi[1]`, `0x8[r6] == mosi[3]`, `0xc[r6] == mosi[7]`, `0xe[r6] == mosi[9]`, `0x10[r6] == mosi[11]`). Every abort branch targets shared zero-return stub **`0xFFF27214`** (`66 0e ef ef 38 47 0d`: `mov #0, r14; mov.L r14, r15; bra.W 0xfff27f5f`).

1. **`CMD 0xE2` (`0xFFF2721B`) & `CMD 0xE6` (`0xFFF2731B`) — Dual-Bit `0x06` (`== 0x06`) Gate on `mosi[7]`**:
   Both commands extract `mosi[7]` (`0xc[r6]`), store it to `0xc34[r6]`, validate drive parameters, and test `0x06` with **bit-wise AND + equality check (`and #0x6; cmp #0x6; beq`)** — aborting **ONLY when BOTH bit 1 and bit 2 (`0x06`) are simultaneously set (`(mosi[7] & 0x06) == 0x06`)**:
   - **`CMD 0xE2 @ 0xFFF27239`**: `64 6f` (`and #0x6, r15`), `61 6f` (`cmp #0x6, r15`), `20 d7` (`beq.B 0xfff27214`)
   - **`CMD 0xE6 @ 0xFFF27339`**: `64 6f` (`and #0x6, r15`), `61 6f` (`cmp #0x6, r15`), `3a d7 fe` (`beq.W 0xfff27214`)

2. **`CMD 0xC0` (`0xFFF27408`), `CMD 0xE4` (`0xFFF2762C`), & `CMD 0xE8` (`0xFFF277DA`) — Single-Bit `tst #0x6` (`!= 0`) Gate + Soft-Wall Clamps**:
   These commands test their command flag byte with **`tst #0x6, rN; bne`** — aborting immediately if **EITHER bit 1 OR bit 2 (`(byte & 0x06) != 0`)** is set:
   - **`CMD 0xC0 @ 0xFFF274A3`**: reads `mosi[11]` (`0x10[r6] -> 0xc34[r6]`), raw bytes `fd 74 cc 06 3b 6d fd` (`tst #0x6, r12; bne.W 0xfff27214`); clamps mechanical soft-wall targets by **`±0x14` (`±20` pulses)** at `0xFFF27461` (`add #0x14, r5, r4`) and `0xFFF27473` (`add #0xec, r14, r4`); handles mode `0x0A` (`cmp #0xa, r14; beq.B`) at `0xFFF275B0`.
   - **`CMD 0xE4 @ 0xFFF276AB`**: reads `mosi[9]` (`0xe[r6] -> 0xc34[r6]`), raw bytes `fd 74 ce 06 3b 65 fd` (`tst #0x6, r14; bne.W 0xfff27214`); applies **`±0x14` (`±20` pulses)** soft-wall margin at `0xFFF27684` (`add #0x14, r3, r3`) / `0xFFF27696` (`add #0xec, r3, r3`).
   - **`CMD 0xE8 @ 0xFFF27806`**: reads `mosi[3]` (`0x8[r6] -> 0xc34[r6]`), raw bytes `fd 74 ce 06 3b 0a fa` (`tst #0x6, r14; bne.W 0xfff27214`).

### 5.2 Cross-Fleet Verification of the `and #6; cmp #6` vs `tst #6` Rule & Soft-Wall Margins

Raw byte scanning across 9 representative Nikkor Z packages proves that the **`E2`/`E6` (`== 0x06`) vs `C0`/`E4`/`E8` (`!= 0`) bitmask split is an invariant Nikon Z mount protocol rule across every STM, Dual-STM, and SSVCM lens**, while the mechanical soft-wall margin scales directly with actuator encoder resolution (`±2..6` counts on linear Hall SSVCM lenses vs `±20..106` microsteps on STM lenses):

| Lens (`Unpacked Bank`) | `CMD 0xE2` (`and#6; cmp#6`) | `CMD 0xE6` (`and#6; cmp#6`) | `CMD 0xC0` (`tst#6`) | `CMD 0xE4` (`tst#6`) | `CMD 0xE8` (`tst#6`) | Soft-Wall Margin (Pulses/Counts) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `50mm f/1.2 S` (`seg0 @ 0xFFF00000`) | `0xFFF27239` (`646f616f20d7`) | `0xFFF27339` (`646f616f3ad7`) | `0xFFF274A3` (`fd74cc06`) | `0xFFF276AB` (`fd74ce06`) | `0xFFF27806` (`fd74ce06`) | **`±0x14` (`±20`)** |
| `MC 105mm f/2.8 VR S` (`seg0 @ 0xFFF00000`) | `0xFFF2F034` (`646f616f20d7`) | `0xFFF2F134` (`646f616f3ad7`) | `0xFFF2F29E` (`fd74cc06`) | `0xFFF2F4A6` (`fd74ce06`) | `0xFFF2F601` (`fd74ce06`) | **`±0x14` (`±20`)** |
| `70-200mm f/2.8 VR S` Gen I (`seg1 @ 0xFFE00000`) | `0xFFF70C0A` (`646f616f20d7`) | `0xFFF70D0B` (`646f616f3ad6`) | `0xFFF70E75` (`fd74cc06`) | `0xFFF71080` (`fd74ce06`) | `0xFFF711DE` (`fd74ce06`) | **`±0x28` (`±40`)** |
| `70-200mm f/2.8 VR S II` Gen II (`seg1 @ 0xFFE00000`) | `0xFFF0F4F3` (`646f616f20d4`) | `0xFFF0F5F7` (`646f616f3ad0`) | `0xFFF0F76F` (`fd74c306`) | `0xFFF0F962` (`fd74c506`) | `0xFFF0FB3C` (`fd74ce06`) | **`±0x06` (`±6`)** |
| `400mm f/2.8 TC VR S` (`seg1 @ 0xFFE00000`) | `0xFFE3508D` (`646f616f20d7`) | `0xFFE3518D` (`646f616f3ad7`) | `0xFFE352F7` (`fd74cc06`) | `0xFFE354F3` (`fd74ce06`) | `0xFFE3564E` (`fd74ce06`) | **`±0x02` (`±2`)** |
| `600mm f/4 TC VR S` (`seg1 @ 0xFFE00000`) | `0xFFE328C2` (`646f616f20d7`) | `0xFFE329C2` (`646f616f3ad7`) | `0xFFE32B2C` (`fd74cc06`) | `0xFFE32D28` (`fd74ce06`) | `0xFFE32E83` (`fd74ce06`) | **`±0x02` (`±2`)** |
| `800mm f/6.3 VR S` (`seg0 @ 0xFFE00000`) | `0xFFE25FB6` (`646f616f20d7`) | `0xFFE260B6` (`646f616f3ad7`) | `0xFFE26220` (`fd74cc06`) | `0xFFE2641C` (`fd74ce06`) | `0xFFE26577` (`fd74ce06`) | **`±0x04` (`±4`)** |
| `100-400mm f/4.5-5.6 VR S` (`seg1 @ 0xFFE00000`) | `0xFFF52C2E` (`646e616e20dc`) | `0xFFF52D29` (`646e616e3ae1`) | `0xFFF52E8B` (`fd74c206`) | `0xFFF5307E` (`fd74ce06`) | `0xFFF531CF` (`fd74cf06`) | **`±0x56` (`±86`)** |
| `24-120mm f/4 S` (`seg0 @ 0xFFF00000`) | `0xFFF9F729` (`646f616f20d7`) | `0xFFF9F829` (`646f616f3ad7`) | `0xFFF9F993` (`fd74cc06`) | `0xFFF9FB9B` (`fd74ce06`) | `0xFFF9FCF6` (`fd74ce06`) | **`±0x6A` (`±106`)** |

*(Note: At `CMD 0xE8` in `70-200 G1` (`0xFFF70EA9 btst #0, r12; fc 4b 92; mov.L [r2], r9`) and `70-200 G2` (`0xFFF0FB63 btst #0, r14; fc 4b 43`), both pair `btst #0` with the 3-byte RXv2 instruction `fc 4b <reg>` right after `add #4, rN, rM` to select between near/far focus boundary slots.)*

---

## 6. Complete 24-Package Nikkor Z Fleet Matrix

*(All `version_raw` header values encode BCD `0x0121 = 1.21`, `0x0120 = 1.20`, `0x0111 = 1.11`, `0x0110 = 1.10`, `0x0101 = 1.01`; `Slots` reports defined command table entries `sub_ptr != 0`, which equals active `enable=1` entries plus the 2 disabled slots `{0x70, 0x71}`.)*

| # | Lens Slug | Vendor Container | Model ID | BCD Ver | Segment(s) Base & Size | RAM Domain | Active `INTB` | SPI Table VA (`Stride`, `Slots`) | Distinct Features |
| :-: | :--- | :--- | :---: | :---: | :--- | :--- | :-: | :--- | :--- |
| 1 | `70-200_f2.8_S_I_v1.21` | `MLZ70_200F28VRS0121.bin.quad.img` | `0x10` | `0x121` | `seg0`: `0xFFF00000` (`1,007,616 B`)<br>`seg1`: `0xFFE00000` (`1,835,008 B`) | SRAM0 `0xC000`<br>EXRAM `0x80A000` | `42`<br>`15` | `0xFFECD770` (`24B`, `65`) | Dual-STM, `4×72,669 B` blocks, `seg0` `22` `0x55` cmds, `±40` soft-wall |
| 2 | `70-200_f2.8_S_II_v1.10` | `MLZ70_200F28VRSII_0110.bin.quad.img` | `0x36` | `0x110` | `seg0`: `0xFFF00000` (`950,272 B`)<br>`seg1`: `0xFFE00000` (`1,802,240 B`) | SRAM0 `0x18400`<br>EXRAM `0x80A000` | `31`<br>`19` | `0xFFEEBEE4` (**`28B`**, **`67`**) | Dual-SSVCM, `0xB0`+`0xB1`, `seg0` `10` `0x55` cmds + `20B` `0xFE`, `±6` soft-wall |
| 3 | `100-400_f4.5-5.6_S_v1.10` | `MLZ100_400F45_56VRS0110.bin.quad.img` | `0x1C` | `0x110` | `seg0`: `0xFFF80000` (`425,984 B`)<br>`seg1`: `0xFFE00000` (`1,802,240 B`) | SRAM0 `0x18400`<br>EXRAM `0x80A000` | `21`<br>`19` | `0xFFED9CC0` (`24B`, `65`) | Dual-MCU, `seg0` `SCI5` `0x55` 2-path, `±86` soft-wall |
| 4 | `180-600_f5.6-6.3_v1.01` | `MLZ180_600F56_63VR0101.bin.img` | `0x2A` | `0x101` | `seg0`: `0xFFF80000` (`425,984 B`)<br>`seg1`: `0xFFE00000` (`1,802,240 B`) | SRAM0 `0x18400`<br>EXRAM `0x80A000` | `21`<br>`19` | `0xFFEC8304` (`24B`, **`66`**) | Dual-MCU, adds **`0xB0`** |
| 5 | `400_f2.8_TC_VR_S_v1.10` | `MLZ400F28TCVRS0110.img` | `0x1E` | `0x110` | `seg0`: `0xFFF80000` (`425,984 B`)<br>`seg1`: `0xFFE00000` (`1,835,008 B`) | SRAM0 `0x18400`<br>EXRAM `0x80A000` | `26`<br>`20` | `0xFFE1BF60` (`24B`, `65`) | Dual-SSVCM C² quintic, built-in `1.4x TC` + thermal table, `±2` soft-wall |
| 6 | `600_f4_TC_VR_S_v1.10` | `MLZ600F4TCVRS0110.img` | `0x25` | `0x110` | `seg0`: `0xFFF80000` (`425,984 B`)<br>`seg1`: `0xFFE00000` (`1,835,008 B`) | SRAM0 `0x18400`<br>EXRAM `0x80A000` | `26`<br>`20` | `0xFFE19808` (`24B`, `65`) | Dual-SSVCM C² quintic, built-in `1.4x TC`, `±2` soft-wall |
| 7 | `800_f6.3_VR_S_v1.10` | `zfw800_MLZ800F63VRS0110.bin.img` | `0x20` | `0x110` | `seg0`: `0xFFE00000` (`1,802,240 B`) | EXRAM `0x80A000` | `16` | `0xFFE0F6FC` (`24B`, `65`) | Active `SCI1` `0x55` (`0xFFE15312`, unbundled sub-MCU), `±4` soft-wall |
| 8 | `800_f6.3_VR_S_v1.11` | `zfw800_MLZ800F63VRS0111.bin.img` | `0x20` | `0x111` | `seg0`: `0xFFE00000` (`1,802,240 B`) | EXRAM `0x80A000` | `16` | `0xFFE0F6FC` (`24B`, `65`) | Active `SCI1` `0x55` (unbundled sub-MCU) |
| 9 | `24-70_f2.8_S_v1.20` | `MLZ24_70F28S0120.img` | `0x0D` | `0x120` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x19A34` | `14` | `0xFFF41014` (`24B`, `65`) | Single-MCU standard zoom |
| 10 | `24-120_f4_S_v1.10` | `MLZ24_120F4S0110.bin.quad.img` | `0x1F` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x1A270` | `11` | `0xFFF5D188` (`24B`, `65`) | Single-MCU zoom, `±106` soft-wall |
| 11 | `24-120_f4_S_v1.20` | `MLZ24_120F4S0120.bin.quad.img` | `0x1F` | `0x120` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x1A278` | `11` | `0xFFF5D188` (`24B`, `65`) | Single-MCU zoom revision |
| 12 | `24-50_f4-6.3_v1.01` | `MLZ24_50F4_630101.img` | `0x16` | `0x101` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x1A278` | `11` | `0xFFF38F20` (`24B`, `65`) | Single-MCU compact zoom |
| 13 | `28-75_f2.8_v1.10` | `MLZ28_75F28_0110.bin.img` | `0x23` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x1A73C` | `11` | `0xFFF7052C` (`24B`, `65`) | Single-MCU f/2.8 zoom |
| 14 | `DX16-50_f3.5-6.3_VR_v1.01` | `MLZDX16_50F35_63VR0101.img` | `0x0B` | `0x101` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x1952C` | `16` | `0xFFF31CC8` (`24B`, **`62`**) | DX VR zoom; omits `0x30`, `0x31`, `0xEB` |
| 15 | `20_f1.8_S_v1.10` | `MLZ20F18S0110.img` | `0x11` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x19100` | `12` | `0xFFF0793C` (`24B`, `65`) | Single-MCU S-line prime |
| 16 | `24_f1.8_S_v1.10` | `MLZ24F18S0110.img` | `0x0F` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x18ED8` | `12` | `0xFFF078EC` (`24B`, `65`) | Single-MCU S-line prime |
| 17 | `35_f1.8_S_v1.10` | `MLZ35F18S0110.img` | `0x04` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x18ED4` | `12` | `0xFFF07C58` (`24B`, `65`) | Single-MCU S-line prime |
| 18 | `35_f1.4_v1.01` | `MLZ35F140101.img` | `0x33` | `0x101` | `seg0`: `0xFFF00000` (`884,736 B`) | SRAM0 `0x19D30` | `11` | `0xFFF0C3B4` (`24B`, **`66`**) | `864 KB` flash, adds **`0xB0`** |
| 19 | `50_f1.8_S_v1.10` | `zfw50_110.img` | `0x09` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x18DB0` | `12` | `0xFFF073B0` (`24B`, `65`) | Single-MCU S-line prime |
| 20 | `50_f1.8_S_v1.11` | `zfw50_111.img` | `0x09` | `0x111` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x18DB0` | `12` | `0xFFF073B0` (`24B`, `65`) | Single-MCU S-line prime revision |
| 21 | `50_f1.4_v1.01` | `MLZ50F140101.img` | `0x34` | `0x101` | `seg0`: `0xFFF00000` (`884,736 B`) | SRAM0 `0x19D30` | `11` | `0xFFF0C33C` (`24B`, **`66`**) | `864 KB` flash, adds **`0xB0`** |
| 22 | `50_f1.2_S_v1.10` | `MLZ50F12S0110.img` | `0x15` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x18C00` | `13` | `0xFFF0E974` (`24B`, `65`) | `4 kHz` `CMT3`, `±20` soft-wall |
| 23 | `85_f1.8_S_v1.10` | `MLZ85F18S0110.img` | `0x0E` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x19100` | `12` | `0xFFF07978` (`24B`, `65`) | Single-MCU S-line prime |
| 24 | `MC105_f2.8_VR_S_v1.10` | `MLZMC105F28VRS0110.img` | `0x18` | `0x110` | `seg0`: `0xFFF00000` (`851,968 B`) | SRAM0 `0x1AFA4` | `17` | `0xFFF119E8` (`24B`, `65`) | Dual-focus macro VR, `±20` soft-wall |

---

## 7. Verified Discrepancy Errata (Legacy Reports & Earlier Tooling vs Raw ROM)

1. **MCU ISA & Flash Layout (`Z_LENS_COMM_AND_FW_REPORT_2026-08-26.md`)**:
   - **Legacy Claim**: Misidentified the `62 FB` / `fb 62` byte pattern as ARM Thumb-16 `LDR r6, [r5, #imm5] + STR r3, [r7, #0x2C]`, classified the MCU as Fujitsu FM3 (`MB9BF`) mirroring flash at top of 4 GB, and treated `0x0080A030..0x0081D590` (`0x81xxxx`) as a "second MCU flash".
   - **ROM Reality**: Every image in the fleet is **Renesas RXv2 (`RX65N/RX651`)** machine code (`fb 62 <u32>` = `mov.L #imm32, r6`). `DTBL` and `BTBL` at `0xFFE00C00..0xFFE00CA8` prove `0x0080A000..0x0081F5A4` is **RX65N Extended SRAM (`EXRAM`)** mapped as `.bss`/`.data` on `seg1` (`0xFFE00000`), while `0xFFE00CA8` / `0xFFF00CA8` is the RX `INTB` vector table.
2. **Container Record Framing & "Mask ROM" Claim (`MLZ180-600-DESIGN.md`, `MLZ-STM-DRIVE.md`, `MLZ-SSVCM-DRIVE.md`, `MLZRE.md`)**:
   - **Legacy Claim**: Interpreted `262 B` (`0x106 B`) container chunks (`04 01 00 idx bank ff`) as "14-bank 格栅槽阵" optical tables, `fb 62 <u32>` as a patch operator, `0x18400` as a mask-ROM hook target, `SY/SC/AF/AC/VR/VC` table entries as hardware register windows, and concluded that STM/SSVCM drive loops lived inside an inaccessible mask ROM with zero executable functions in the update file.
   - **ROM Reality**: Each `0x106 B` unit is a `[u16 0x0104][u32 target_va][256 B flash page]` transfer record. Reassembling those pages recovers **complete, self-contained Renesas RXv2 flash images** containing `ResetPRG`, full ISR vector tables (`INTB`), `RSPI2`/`MTU`/`ADC` drivers, 6-segment quintic solvers (`400TC/600TC`), dual-SSVCM PID (`70-200 G2`), and `4 × 72,669 B` STM tables (`70-200 G1`). `.bss` base `0x18400` is SRAM on `seg0` (`BTBL @ 0xFFF00C58` / `0xFFF80C58`).
3. **Hardcoded `24 B` Subcommand Stride (`scan_all_z_cmd_tables.py`)**:
   - **Script Defect**: Line 48 hardcoded `24 * sidx`, corrupting subcommand entry extraction for `70-200mm f/2.8 VR S II` (`0xFFEEBEE4`), which uses **`28 B` (`mul #0x1c`) descriptors** incorporating `+0x18` payload limits.
4. **Header BCD Version Decoding**:
   - Header `version_raw` words (`0x0121`, `0x0120`, `0x0111`, `0x0110`, `0x0101`) are packed **BCD** (`1.21`, `1.20`, `1.11`, `1.10`, `1.01`), rather than hexadecimal integers.
5. **Rizin `rizin -a rx` Disassembler Artifacts**:
   - Emits spurious `.UB` suffixes on 32-bit register ALU operations (`add.UB`, `sub.UB`, `mul.UB`, `min.UB`, `max.UB`, `tst.UB`, `cmp.UB`).
   - Displays unscaled word/long displacement indices (e.g., `0x2[r1]` on `mov.L` accesses byte offset `+0x08`).
   - Fails to decode Renesas RX wrap-around 1-byte short branches where 3-bit displacement `{0, 1, 2}` encodes `{+8, +9, +10}`: e.g., `0x09` (`bra.S +9`, verified at `70-200 G2` `0xFFEFC70C -> 0xFFEFC715`) and `0x11` (`beq.S +9`, verified at `50/1.2` `0xFFF27496 -> 0xFFF2749F` and `70-200 G2` `0xFFF0F762 -> 0xFFF0F76B`), as well as RXv2 3-byte `0xFC 0x4B <reg>` instructions after `btst #0` (`70-200 G1 @ 0xFFF70EAB`, `70-200 G2 @ 0xFFF0FB65`). Always cross-check raw bytes for critical control branches.
