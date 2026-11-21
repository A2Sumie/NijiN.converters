# BORYOZA E→Z V2.24 (`BORYOZA_V224.PUB`) Build Manifest, Verification & Known Limitations / 构建指纹、验证矩阵与已知局限性

**[English](#english) · [简体中文](#简体中文)**

---

## English

### 1. Build Artifacts & Hardware Baseline (`V2.24` / `LF 2.24`)

| Artifact | Path / Origin | Size / Metric | Digest / Invariant |
|---|---|---|---|
| USB MSC Upgrade Package `.PUB` | `se-nz/build/BORYOZA_V224.PUB` | `297,984 B` (`582` × `512 B` sectors) | `99ae594983db3055c349497819ef69a6dd9b23c04c47a051766f76a936f0fbc7` |
| Raw APP Flash Image `.bin` | `se-nz/build/BORYOZA_V200.bin` | `297,872 B` (`0x00010000 .. 0x00058BF0`) | `c795787dc5290e7b2ab32a88dcf878d1f2649806c4afbd6e4d74f8ad610fdeb2` |
| Linked Cortex-M4F `.elf` | `se-nz/build/BORYOZA_V200.elf` | `.text=290,932 B`, `.ramfunc=2,820 B`, `.data=16 B`, `.bss=6,360 B`, `.optics_sram2=63,564 B`, `.diag_sram3=23,552 B`, `.retram_noload=4,024 B` | `MSP=0x1FFFFF00`, `Reset_Handler=0x00010281`, `program_end=0x00058BF0 <= 0x00060000` |
| Real-Time & Packer Audit | `se-nz/build/pack_manifest.json` | `116` no-FPU functions / `42` RAM ISR closures / `5` EFM SRAM closures | `nofpu_violations=0`, `closure_violations=0`, `.readme_ver=18020b1a`, `0` 64-bit soft division |
| On-Camera Telemetry (`0x807B`) | `FE 50-150mm F2 GM` (`Z=0x0960`) | `1.5 Mbps` UART + `60 Hz` VSync | `311` AF commands, `0` stalls, `0` RX/CRC errors, `0xE6` residual `-2 se`, `50 -> 78 -> 150 mm` zoom stable |
| On-Camera Telemetry (`0xC56E`) | `Sigma 150-600mm F5-6.3 DG DN OS | Sports` (`Z=0x09C5`) | `1.5 Mbps` UART + `120 Hz` VSync | `1,099` AF commands over `112.4 s`, `0` stalls, `0` RX/CRC errors, `0` false zoom MF triggers, `0xE6` 3D-Tracking active |

Reproduce locally from repository root:

```sh
make && make test
```

### 2. Key Architectural Changes (`V2.23` → `V2.24`)

1. **Expanded On-Board Optical Database**:
   - `276` lens catalog entries (`+0xC13A`), `58` delta-nibble 81-point spectral transmission curves, and `92` sparse/Huffman-compressed 3D `CMD 0x24` A/B optical profiles (`290,932 B` `.text + .rodata`, leaving `29,712 B` below the `0x00060000` APP ceiling).
2. **EXPEED 6 (`Z6`/`Z7`/`Z50`/`Z6II`/`Z7II`/`Z30`/`Zfc`) & EXPEED 7 (`Z8`/`Z9`/`Zf`/`Z6III`) Protocol Hardening**:
   - Body generation fingerprinting and dynamic `CMD 0x20/00` VSync tracking (`lens_session_apply_body_vsync_hz`), keeping `60 Hz` bodies locked at `60 Hz` E-mount cadence while enabling `120 Hz` on EXPEED 7 with `tx08[0x0C]` bit-7 capable lenses (`Sigma HLA`, `Tamron VXD`, `Viltrox`).
   - Populated `CMD 0xC4/00` (`22 B`) MISO snapshot, `CMD 0xC1` (`mosi[20] & 0x06` brake + `mosi[15..18]` soft wall), `CMD 0xEC` (`mosi[9] & 0x06` brake), `CMD 0xEE` trajectory lookahead, progress-aware `CMD 0xE8` timeout (`900 ms` stall window), general geared-DC backlash guard, and `MAX_VPPS` unlocking across high-speed linear actuators.
   - Relocated active Ping/Pong Journal rotation to `JNL_PONG` (`0x78000`, `LBA 842..857`) ↔ `JNL_A` (`0x7C000`, `LBA 874..889`) with persistent `FX/DX` crop LRU caching, preserving legacy `JNL_B` (`0x7E000`, `LBA 890..905`) read-only near the `0x7FFF8` security bytes.
3. **Field-Verified AF & Varifocal Refinements**:
   - Disabled unsolicited `0x1D` idle electronic-parfocal trims in production (`parfocal_research_enable = 0` by default; opt-in via `af_servo_set_parfocal_research_mode()`), retaining active-AF (`0xE4`/`0xE6`) zoom target reprojection.
   - Suppressed `30–60 Hz` varifocal cam-step false MF-bar triggers on `120 Hz` zoom lenses (`confirm_cam_zoom_mf_dir`) while keeping `zoom_guard_vsyncs` decay active during `AF-C` / `3D-Tracking` so `500 Hz` `SPI1` hotline velocity interpolation (`MIN_SAMPLE_DT_U32 = 18000` ticks = `6.0 ms`) stays responsive immediately after zooming.

### 3. Known Limitations (`V2.24`)

1. **Continuous AF (`AF-C` / `3D-Tracking`) During Rapid Mechanical Zooming**:
   - Rapidly rotating the zoom ring while actively tracking (`0xE4` / `0xE6` / `0xEE`) shifts mechanical cam limits (`lim_far_se` / `lim_near_se`), exit pupil position, and defocus sensitivity (`KZ`) asynchronously (`30–60 Hz` internal zoom ADC rate vs `60/120 Hz` VSync). Active target reprojection maintains drive continuity, yet transient focus lag or brief defocus occurs mid-sweep until zoom motion settles.
2. **Idle Electronic Parfocal Off by Default**:
   - Automatic `dist_code_x` closed-loop focus trimming during idle zoom sweeps is disabled in default production builds (`parfocal_research_enable = 0`) because `1/64`-octave `ID 0x05` distance quantization and multi-frame phase lag on floating-group zooms can cause minor step hunting during slow manual zoom sweeps. Re-acquire focus (`AF-S` / `AF-ON`) after zooming, or enable `af_servo_set_parfocal_research_mode()` explicitly in custom/SIL builds.
3. **Sony First-Party XD Lenses Locked to `1.5 Mbps + 60 Hz` VSync**:
   - Recent Sony GM/G lenses (including `FE 50-150mm F2 GM`, `0x807B`) advertise `tx08[0x0C] = 0x12` (`60 Hz` baseline) unless the host completes the extended 13-byte `CMD 0x01` capability mask, `CMD 0x4E / 0x4C / 0x4B / 0x0B` high-rate negotiation, `CMD 0x0A` `ID 0x05` group pruning, and `ID 0x04[6]` frame-period scaling. Driving `120 Hz` VSync without that 4-step negotiation overruns the lens's internal `16.67 ms` 18-group curve budget during zoom; V2.24 runs these lenses at rock-solid `1.5 Mbps + 60 Hz` (`0` stalls, `0` zoom disconnects).
4. **Stacked Smart Mount Converters (`LA-FE1` / Secondary Bridge MCUs)**:
   - Daisy-chaining secondary electronic mount adapters (such as Nikon F → Sony E `LA-FE1`, `0x6006`) onto `SE-NZ` requires `80–250 ms` secondary MCU + F-mount motor cold-boot initialization before answering `CMD 0x01`, exceeding the E-mount primary handshake window and entering passive fallback. Direct Sony E-mount lenses (first-party and third-party) are the supported hardware topology.
5. **Nikon Z Teleconverters & Dedicated Power-Zoom (`PZ`) Controls**:
   - Nikon Z `TC-1.4x` / `TC-2.0x` teleconverters physically protrude into the rear mount throat and cannot mount behind an E-to-Z adapter (`CMD 0x31` is answered safely as a no-op); Sony E-mount `SEL14TC` / `SEL20TC` (`0x8138` / `0x8139`) on compatible E-mount telephotos are supported. Dedicated E-mount Power-Zoom rocker speed commands beyond standard focal telemetry are out of scope.
6. **APP Flash Capacity Margin (`320 KiB` Partition Limit)**:
   - `BORYOZA_V224.PUB` uses `297,872 B` of the `327,680 B` (`[0x00010000, 0x00060000)`) APP Flash region (`90.90%`), leaving `29,808 B` before the `0x00060000` USB MSC FAT12 boundary. Adding further uncompressed optical tables requires pruning or tighter entropy coding to stay within `0x0005FFE0`.

---

## 简体中文

### 1. 构建产物与硬件实测基线（`V2.24` / `LF 2.24`）

| 交付产物 | 仓库路径 / 来源 | 体积 / 指标 | SHA-256 / 关键不变量 |
|---|---|---|---|
| USB MSC 升级包 `.PUB` | `se-nz/build/BORYOZA_V224.PUB` | `297,984 B`（`582` 个 `512 B` 扇区） | `99ae594983db3055c349497819ef69a6dd9b23c04c47a051766f76a936f0fbc7` |
| 裸机 Flash 镜像 `.bin` | `se-nz/build/BORYOZA_V200.bin` | `297,872 B`（`0x00010000 .. 0x00058BF0`） | `c795787dc5290e7b2ab32a88dcf878d1f2649806c4afbd6e4d74f8ad610fdeb2` |
| Cortex-M4F 符号镜像 `.elf` | `se-nz/build/BORYOZA_V200.elf` | `.text=290,932 B`, `.ramfunc=2,820 B`, `.data=16 B`, `.bss=6,360 B`, `.optics_sram2=63,564 B`, `.diag_sram3=23,552 B`, `.retram_noload=4,024 B` | `MSP=0x1FFFFF00`, `Reset_Handler=0x00010281`, `program_end=0x00058BF0 <= 0x00060000` |
| 硬实时与打包门禁 | `se-nz/build/pack_manifest.json` | `116` 个禁浮点函数 / `42` 个 RAM ISR 闭包 / `5` 个 EFM SRAM 闭包 | `nofpu_violations=0`, `closure_violations=0`, `.readme_ver=18020b1a`, 零 `64-bit` 软除法 |
| 上机黑匣子实测 (`0x807B`) | `Sony FE 50-150mm F2 GM` (`Z=0x0960`) | `1.5 Mbps` UART + `60 Hz` VSync | `311` 次 AF 指令、`0` 失步、`0` 通信误码、`0xE6` 残差 `-2 se`、`50 -> 78 -> 150 mm` 变焦零掉线 |
| 上机黑匣子实测 (`0xC56E`) | `Sigma 150-600mm F5-6.3 DG DN OS | Sports` (`Z=0x09C5`) | `1.5 Mbps` UART + `120 Hz` VSync | 连续 `112.4 s` 共 `1,099` 次 AF 指令、`0` 失步、`0` 通信误码、变焦 `0` 假 MF 标尺、`0xE6` 3D 追焦顺畅 |

### 2. V2.23 → V2.24 核心演进要点

1. **片上光学与光谱数据库扩容**：升级至 `276` 项镜头目录（新增 `0xC13A` 等映射）、`58` 条 81 点光谱透过率曲线及 `92` 组稀疏/Huffman 压缩 3D `CMD 0x24` A/B 光学校正表。
2. **EXPEED 6 / EXPEED 7 三代机身协议加固**：增加机身代际指纹与 `CMD 0x20/00` 实时帧率跟随双门控（`E6` 与 `60 Hz` 视频/省电档锁定 `60 Hz`；`E7` 配合 `tx08[0x0C]` bit7 副厂高速头开启 `120 Hz`）；补齐 `CMD 0xC4/00`（`22 B`）快照、`CMD 0xC1/0xEC` 刹停与软墙、`CMD 0xEE` 轨迹前瞻、`CMD 0xE8` 推进感知超时、通用齿轮 DC 马达换向消隙保护及高速线性马达 `MAX_VPPS` 解锁；活跃 `Ping/Pong` 日志迁至 `JNL_PONG`（`0x78000`, `LBA 842..857`）与 `JNL_A`（`0x7C000`, `LBA 874..889`）并持久化 `FX/DX` 画幅 LRU。
3. **变焦与 3D 追焦现场回归收敛**：生产默认关闭闲置变焦电子齐焦微调（`parfocal_research_enable = 0`），保留主动追焦（`0xE4`/`0xE6`）期间的变焦目标重投影；在 `120 Hz` 变焦头中抑制 `30–60 Hz` 凸轮步进引起的虚假 MF 标尺触发，同时允许主动追焦期间正常衰减 `zoom_guard_vsyncs`，确保变焦后 `500 Hz` `SPI1` 热线速度内插（`MIN_SAMPLE_DT_U32 = 18000` ticks = `6.0 ms`）即时恢复。

### 3. 已知局限性（Known Limitations, `V2.24`）

1. **快速拧动变焦环期间的连续追焦（变焦 `AF-C` / `3D-Tracking`）**：
   - 在按住 `AF-ON` 持续追焦的同时快速拧动机械变焦环时，镜头内部凸轮端点（`lim_far_se` / `lim_near_se`）、出瞳位置与散焦灵敏度系数 `KZ` 按镜内 `30–60 Hz` 变焦 ADC 节拍异步跳变，与机身相位检测窗存在跨帧相位差；变焦途中会出现追焦滞后或短暂虚焦，待变焦停稳后恢复正常合焦与跟踪。
2. **空闲状态电子齐焦生产默认关闭**：
   - 非对焦空闲状态下拧动变焦环的自动 `0x1D` 电子齐焦补偿在生产固件中默认关闭（`parfocal_research_enable = 0`），原因是多组浮动对焦变焦头在 `ID 0x05 dist_code_x`（`1/64` 倍频程分档）离散量化与多帧延迟下慢拧变焦会产生残余微步抽动；变焦到位后请重新半按对焦，或仅在主机 SIL / 自定义研究构建中显式调用 `af_servo_set_parfocal_research_mode()` 开启。
3. **Sony 原厂 XD 线性马达镜头固定运行于 `1.5 Mbps + 60 Hz` VSync**：
   - 新一代 Sony 原厂 GM/G 镜头（如 `FE 50-150mm F2 GM`, `0x807B`）默认上报 `tx08[0x0C] = 0x12`（`60 Hz` 基准周期）；原厂机身需完整执行 13 字节 `CMD 0x01` 扩展掩码、`CMD 0x4E / 0x4C / 0x4B / 0x0B` 高帧率协商、`CMD 0x0A` 精简冗余 `ID 0x05` 曲线组以及 `ID 0x04[6]` 帧周期比例字节后，镜内 DSP 才切入 `8.33 ms`（`120 Hz`）预算。V2.24 将此类原厂头锁定在稳定的 `1.5 Mbps + 60 Hz` 模式（实测变焦与追焦零掉线、零失步）；具备 `tx08[0x0C]` bit7 标志的副厂高速头（如 `Sigma HLA`、`Tamron VXD`、`Viltrox`）在 EXPEED 7 机身上正常启用 `120 Hz`。
4. **不支持串接第三方智能转接环（如 `LA-FE1`, `0x6006`）**：
   - 将二级电子转接环（如 Nikon F → Sony E 的 `LA-FE1`）叠接在本转接环前端时，二级桥接 MCU 与前端 F 口马达冷启动自检耗时 `80–250 ms`，超出标准 E 口首轮握手窗口而进入被动回退状态；仅支持直连原生或副厂 E 卡口镜头。
5. **Nikon Z 口增距镜与 E 口电动变焦（`PZ`）控制**：
   - Nikon 原厂 Z 口 `TC-1.4x` / `TC-2.0x` 前组凸出，物理结构上与 E→Z 转接环互斥（`CMD 0x31` 仅作安全空响应）；兼容长焦 E 口镜头搭配 Sony E 口 `SEL14TC` / `SEL20TC`（`0x8138` / `0x8139`）可正常识别。除标准焦距回传外的专用 E 口电动变焦（`PZ`）拨杆变速指令不在当前支持范围内。
6. **`320 KiB` APP Flash 分区余量边界**：
   - `BORYOZA_V224.PUB` 已使用 `[0x00010000, 0x00060000)` `320 KiB` APP 分区中的 `297,872 B`（占比 `90.90%`），距 `0x00060000` FAT12 文件系统边界剩余约 `29.1 KiB`；后续若继续大幅扩充三维光学表，需同步裁剪或提升熵编码压缩率以严守 `0x0005FFE0` 上限。
