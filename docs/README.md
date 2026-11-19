# NijiN.converters Documentation Index / 文档总索引

**[English](#english) · [简体中文](#简体中文)**

This directory contains the architecture overviews, flashing guides, and the complete sanitized **Sony E-mount → Nikon Z-mount (`BORYOZA SE-NZ`, `HC32F460JEUA`)** reverse-engineering, optical, autofocus math, and protocol reference library.

---

## English

### 1. Project & Tooling Guides (`docs/`)

| Document | Scope |
|---|---|
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | `HC32F460JEUA` 5-bank SRAM & Flash partition map, source layering (`00_hal` / `01_transport` / `02_core` / `03_diag_storage`), and real-time static gates |
| [`FLASHING_AND_DIAGNOSTICS.md`](FLASHING_AND_DIAGNOSTICS.md) | Building `.elf` / `.bin` / `.PUB`, contiguous FAT12 USB MSC flashing (`flash_usb_bor.py`), and Blackbox / Journal telemetry extraction |

### 2. Complete E→Z Reference Library (`docs/ez-reference/`)

| Volume | Scope |
|---|---|
| [`PART1_NIKKOR_Z_FLEET.md`](ez-reference/PART1_NIKKOR_Z_FLEET.md) | Native Nikkor Z RXv2 firmware architecture across `24` unpacked lens packages (`21` models): clock tree, `SCI5`/`DMAC` Z-mount transport, `SCI1` inter-MCU `0x55` bridge, servo loops, and `AF` abort bitmasks |
| [`PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md`](ez-reference/PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md) | Nikon Z8 `v3.10`/`v3.12` state machine (`164` descriptors, `600`-entry property bank), `1 kHz SPI1` hotline, Sony `A1 II`/`A9 III` E-mount protocol, third-party E lens deltas (Sigma / Tamron / Samyang / Zeiss / Cosina), and closed-loop coupling |
| [`PART3_EZ_BRIDGE_AND_ERRATA.md`](ez-reference/PART3_EZ_BRIDGE_AND_ERRATA.md) | E-to-Z coordinate & optical translation math, complete `Known Bad` failure catalog, and machine-code-verified errata (`V2.16`–`V2.19`) |
| [`PART4_PDAF_MATH_AND_CONTROL_OPT.md`](ez-reference/PART4_PDAF_MATH_AND_CONTROL_OPT.md) | Z8 `3.12` rational PDAF solver equations, pole paradox diagnostics, damping/velocity envelopes, Cortex-M4F MCU optimization, Canonical Huffman `CMD 0x24`, and delta-nibble spectral compression |
| [`PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md`](ez-reference/PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md) | Complete Sony E / Nikon Z crosswalk for distance/magnification/cam-curve fields, Sony `Bonobo` 5-axis SteadyShot vs Nikon `0xA3`/`BVRCOM`, `Sony2Fp.FocusPosition2` EXIF (`255` sentinel & MF scale decoupling), and 3-generation AF drive commands (`0x1D`/`0x43`/`0x6B`) |
| [`Z8_SONY_AF_ARCHITECTURE_STUDY.md`](ez-reference/Z8_SONY_AF_ARCHITECTURE_STUDY.md) | Comparative disassembly study of Z8 `3.12` + flagship S-Line primes vs Sony `A1 II`/`A9 III` and large-defocus high-aperture capture dynamics |
| [`HC32F460_SE_NZ_DESIGN_SPEC.md`](ez-reference/HC32F460_SE_NZ_DESIGN_SPEC.md) | Full `HC32F460JEUA` clean-room firmware engineering design specification (clocks, GPIO pads, DMA, E/Z wire frames, AF/VR/aperture/optics domains, and verification gates) |
| [`BOOTLOADER_AND_USB_MSC_CONTRACT.md`](ez-reference/BOOTLOADER_AND_USB_MSC_CONTRACT.md) | Stock `64 KiB` Bootloader handoff ABI, `PA9` camera/USB boot mux, QFN48 `13`-pin role transitions, and FAT12 USB MSC continuous-stream writer (`0x5070`) contract |
| [`SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md`](ez-reference/SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md) | End-to-end SE-NZ hardware/protocol/control-law master specification plus compiled `275`-lens catalog and `57`-curve spectral index (Appendices A–D) |
| [`SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md`](ez-reference/SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md) | `V2.23` (`LF 2.23` / `17020b1a`) build manifest, full-time closed-loop electronic parfocal design (`AF-S`/`AF-C`/`Body MF`/`Barrel MF`), and full SIL/MCU/matrix verification ledger |

---

## 简体中文

### 1. 基础架构与刷机指南（`docs/`）

| 文档 | 内容说明 |
|---|---|
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | `HC32F460JEUA` 五段 SRAM 与 Flash 分区规划、源码四层隔离结构与编译/打包静态门禁 |
| [`FLASHING_AND_DIAGNOSTICS.md`](FLASHING_AND_DIAGNOSTICS.md) | 本地编译 `.elf` / `.bin` / `.PUB`、连续簇 USB MSC 刷写（`flash_usb_bor.py`）与黑匣子/日志读取说明 |

### 2. 完整 E→Z 实现与协议参考文库（`docs/ez-reference/`）

| 卷目 | 内容说明 |
|---|---|
| [`PART1_NIKKOR_Z_FLEET.md`](ez-reference/PART1_NIKKOR_Z_FLEET.md) | Nikkor Z 原厂 RXv2 镜头固件家族全景架构（`24` 份解包镜像 / `21` 款镜头）：内存映射、时钟树、`SCI5`/`DMAC` 与 `AF` 中止掩码 |
| [`PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md`](ez-reference/PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md) | Nikon Z8 `v3.10`/`v3.12` 控制状态机（`164` 描述符、`600` 项 Property）、`1 kHz SPI1` 热线、Sony `A1 II`/`A9 III` E 口协议及各副厂品牌差异矩阵 |
| [`PART3_EZ_BRIDGE_AND_ERRATA.md`](ez-reference/PART3_EZ_BRIDGE_AND_ERRATA.md) | E→Z 桥接方程、全谱系 `Known Bad` 故障工程目录及经机器码反证的历史勘误总汇 |
| [`PART4_PDAF_MATH_AND_CONTROL_OPT.md`](ez-reference/PART4_PDAF_MATH_AND_CONTROL_OPT.md) | Z8 `3.12` 有理式 PDAF 求解器数学拆解、极点悖论定位、Cortex-M4F 热点优化与 Canonical Huffman 光谱/三维光学表压缩 |
| [`PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md`](ez-reference/PART5_DISTANCE_IBIS_AND_AF_CROSSWALK.md) | Sony E / Nikon Z 全链路距离与倍率字段、五轴防抖（`CamCurve/fc_pitch` vs `Property 0xA3` & `BVRCOM`）、`Sony2Fp` EXIF `255` 与 MF 标尺分离成因及三代 AF 驱动指令（`0x1D`/`0x43`/`0x6B`）对照 |
| [`Z8_SONY_AF_ARCHITECTURE_STUDY.md`](ez-reference/Z8_SONY_AF_ARCHITECTURE_STUDY.md) | Nikon Z8 `3.12` + 旗舰 S-Line 与 Sony `A1 II`/`A9 III` 双端对焦架构机器码级对比研究 |
| [`HC32F460_SE_NZ_DESIGN_SPEC.md`](ez-reference/HC32F460_SE_NZ_DESIGN_SPEC.md) | `HC32F460JEUA` E→Z 净室固件完整工程设计规格书（硬件抽象、Z/E 逐字节帧合同、AF/VR/光圈/光学张量四大业务域） |
| [`BOOTLOADER_AND_USB_MSC_CONTRACT.md`](ez-reference/BOOTLOADER_AND_USB_MSC_CONTRACT.md) | 原厂 `64 KiB` Bootloader 交接合同、`PA9` 启动分支、QFN48 管脚分阶段归属与 USB MSC FAT12 连续扇区写入状态机 |
| [`SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md`](ez-reference/SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md) | SE-NZ 全生态协议、控制律与机身兼容总规格书（附 `275` 项镜头目录与 `57` 条光谱映射完整附录 A–D） |
| [`SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md`](ez-reference/SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md) | `V2.23`（`LF 2.23` / `17020b1a`）最终构建指纹、全时闭环电子齐焦架构与全套自动化验证台账 |
