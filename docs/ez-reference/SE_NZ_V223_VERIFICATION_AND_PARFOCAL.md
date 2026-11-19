# BORYOZA E→Z V2.23（`BORYOZA_V223.PUB`）固件构建与全闭环验证报告

> V2.23 交付版本入口为 [`README.md`](../../README.md)，全系统目录与附录见 [`SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md`](SE_NZ_ECOSYSTEM_AND_LENS_CATALOG.md)。

## 1. 交付物与构建摘要

本工程在 `se-nz/` 下基于 `HC32F460JEUA`（Cortex-M4F @ `192 MHz`、`512 KiB` Flash、`192 KiB` 五段 SRAM）完成 `BORYOZA_V223.PUB`（`LF 2.23`、`.readme_ver = 17 02 0b 1a`）全量编译、打包门禁、`16/16` 全闭环测试套件及四轮对抗审查（Break-It Round-4 `0 P0 / 0 P1 / 0 P2`）签字。

| 交付产物 | 仓库路径 / 来源 | 尺寸 / 指标 | SHA-256 / 关键参数 |
|---|---|---|---|
| 刷机升级包 `.PUB` | `se-nz/build/BORYOZA_V223.PUB` | `189,440 B`（`370` 个 `512 B` 扇区） | `7fd7b14693d6a9dedb4e558998ab95af5eec467dccc8828f54f63bc8d436c1ec` |
| 裸机 Flash 载荷 `.bin` | `se-nz/build/BORYOZA_V200.bin` | `189,208 B`（`0x10000 .. 0x3E3F0`） | `e6bcf0cd1037efdad17f16349df0b8301a0112bb1c50caab53872c15cfbc7190` |
| MCU 符号镜像 `.elf` | `se-nz/build/BORYOZA_V200.elf` | `.text=182,332 B`, `.ramfunc=2,764 B`, `.bss=5,976 B`, `.optics_sram2=63,564 B`, `.retram_noload=3,948 B` | `MSP=0x1FFFFF00`, `Reset_Handler=0x00010281`, `program_end=0x0003E3F0` |
| 主机 ASan+UBSan 核心库 | `se-nz/build/libboryoza_core.dylib` | macOS `arm64` 动态库 | `063187ba9853ccac2af9e2d25638b06b4782f21c60048b7bcd0c007eb291e4e7`，与 MCU 共享同一份 `src/02_core/*.c` |
| 打包与实时门禁清单 | `se-nz/build/pack_manifest.json` | `107` 个无浮点函数 / `42` RAM ISR 闭包 / `5` EFM SRAM 闭包 | `nofpu_violations=0`, `closure_violations=0`, `.readme_ver=17020b1a`, 零 `64-bit` 软件除法 |
| V2.22 实机黑匣子证据 (`0x807B`) | `LBA 858..905 USB MSC 实机遥测样本 (0x807B)` | `47/256` 黑匣子 + `4` 条双扇区日志（`0` 损坏） | `HS_TIMEOUT_FALLBACK` (`slot 35`) -> POR `BOOT_OK` (`slot 36`) -> `Z_FIRST_WIN 128.1 ms`, `E_LINK_UP 489.9 ms`, `1,561` 次 AF 指令, `0` 失步/误码 |
| V2.22 实机黑匣子证据 (`0xC56E`) | `LBA 858..905 USB MSC 实机遥测样本 (0xC56E)` | `38/256` 黑匣子 + `3` 条双扇区日志（`0` 损坏） | `120 Hz` 全通, `1,746` 次 AF 指令, `0` 失步/误码 |

复现构建命令：

```sh
make && make test
```

## 2. 存储空间与五段 SRAM 实测分布

### 2.1 Flash 分区（`512 KiB` = `0x00000000 .. 0x0007FFFF`）

| 物理区间 | 分区角色 | 实测占用 / 容量预算 | 关键约束验证 |
|---|---|---|---|
| `0x00000000 .. 0x0000FFFF` | 原厂 USB MSC Bootloader + OTP 身份区 | `64 KiB` | BL SCSI 仅擦写 `>= 0x10000` 扇区，`0x0000..0xFFFF` 原样保持 |
| `0x00010000 .. 0x00010FFF` | `.isr_vector` | `4,096 B` | `[0]=0x1FFFFF00`, `[1]=0x00010281` |
| `0x00011000 .. 0x00011003` | `.readme_ver` | `4 B` | `{0x17, 0x02, 0x0B, 0x1A}` (`LF 2.23`) |
| `0x00011008 .. 0x0003E3F0` | APP 代码与压缩光学数据库（`275` 款镜头、`57` 条光谱、`25` 组 `CMD 0x24` `63,477 B`） | `189,208 B` / `327,680 B` (`57.74%`) | `program_end = 0x0003E3F0 <= 0x00060000`（`320 KiB` APP 分区绝对上限，余量 `138,256 B`） |
| `0x00060000 .. 0x00079FFF` | USB MSC FAT12 保留区 | `0 B` 占用 | 与 APP 分区物理隔离 |
| `0x0007A000 .. 0x0007BFFF` | 遥测与飞行记录区（`LBA 858..873`） | `8,192 B` (`256 × 32 B`) | 反向抗撕裂扫描、单次滚动复用末 `8` 条、`s_bb_flash_degraded` 坏块熔断保护 APP 区 |
| `0x0007C000 .. 0x0007FFFF` | 双扇区原子日志（Ping/Pong） | `2 × 8,192 B` | `generation + CRC32` 原子切换，`s_jnl_flash_degraded` 坏块熔断，配合 `Ret_SRAM` 热启动跳过重扫 |

### 2.2 五段 SRAM 分区（`192 KiB`）

| SRAM 分段 | 物理区间 | 驻留对象 | 实测字节 / 容量 |
|---|---|---|---|
| `SRAMH` | `0x1FFF8000 .. 0x1FFFFFFF` (`32 KiB`) | `.ram_vectors` (`1,024 B`) + `.ramfunc` (`2,764 B`) + `.dma_sramh` (`9,828 B`) + `MSP` 主栈 | `13,616 B` 静态 + 主栈 / `32,768 B` |
| `SRAM1` | `0x20000000 .. 0x2000FFFF` (`64 KiB`) | `.data` (`8 B`) + `.bss` (`5,976 B`，含 `adapter_ctx_t`、`s_crc32_nib_lut`) | `5,984 B` / `65,536 B` (`9.13%`) |
| `SRAM2` | `0x20010000 .. 0x2001FFFF` (`64 KiB`) | `.optics_sram2`（`optics_workspace_t` + 双缓冲 `CMD 0x24` A/B + 快照池） | `63,564 B` / `65,536 B` (`97.00%`) |
| `SRAM3` | `0x20020000 .. 0x20026FFF` (`28 KiB`) | `.diag_sram3`（`512` 条 `32 B` 实时环形追踪 `s_diag_ring`） | `16,384 B` / `28,672 B` (`57.14%`) |
| `RetRAM` | `0x200F0000 .. 0x200F0FFF` (`4 KiB`) | `.retram_noload`：`48 B` `retram_warm_hdr_t` + `3,872 B` `ds13_words` + `28 B` `retram_state_t` | `3,948 B` / `4,096 B` (`96.39%`) |

## 3. V2.23 全时闭环电子齐焦架构与对抗闭环要点

1. **全对焦模式零标定闭环（`af_servo_bridge.c` / `lens_session.c` / `boryoza_types.h`）**：
   - 将 V2.21 原“非机身 MF”限制放开为全模式使能，覆盖 **`AF-S`**、**`AF-C`**、**`机身 MF`（`MOSI20[2] = 0x06` / `MOSI20[0x2B..0x2C] = 0x02`）** 与 **`镜身 MF`（`id05.is_mf_switch = 1`）**，通过 `sync_parfocal_anchor()` -> 薄透镜前馈 `compute_parfocal_ff_cz()` -> `ID 0x05 dist_code_x` 对数分档残差闭环 `compute_parfocal_cl_delta_cz()` -> `maybe_queue_parfocal_trim()`，在零标定条件下跨任意变焦分段、非单一 MFD 与非线性凸轮实时追平物距 `<= 1` 分档；停稳后 `ID 0x04[9]` 立即回到 `0x81`（释放 `MF` 对焦峰值 / 放大 / 景深尺，或保留镜身 MF 下被忽略的 `0x1D` 且 `16 VSyncs` 内零 `AF_STALL_ABORT`）。
   - 使用 `id05_pair_state_t`（`id05_fresh_since_06`）严格区分同帧配对与分离到达的 `ID 0x06 / ID 0x05`，并使能 `active_parfocal_cl_cooldown()`（`60 Hz = 2 VSyncs`, `120 Hz = 4 VSyncs`），防止变焦传输延迟导致的过冲震荡；`same_cmd` 重发与 `can_run_parfocal_dist_x_cl()` 门禁防止冷却计数器被重复覆盖或无 `ID 0x05` 反馈时卡滞。
2. **制动滑行、`b6_flags = 0` 到位与多品牌变焦端点边界（`af_servo_bridge.c` / `aperture_controls.c` / `main.c`）**：
   - `issue_controlled_brake()` 与 `update_e0_brake_and_coast_state()` 通过 `parfocal_e0_coast_rem`（`16 VSyncs`）在 `C0 -> E0` 惯性滑行期间屏蔽重复空闲 `CMD 0xE0` 误提前锚定；`af_servo_build_id04_subrecord()` 在 `机身 MF` 触发 `0x1C` 制动时若 `hw_moving != 0U || settle_frames_rem != 0U` 则维持 `ID 0x04[9] = 0x83U`，使原厂线性马达可靠受理停车。
   - `refresh_cam_endpoints_and_jacobian()` 将 `fixed_catalog_cam` 收窄为 `!is_zoom_lens`，使 `0xC56F` / `0xC13D` / `0xC142` / `0xC136` / `0x0019` 等变焦镜头随实时 `lim_far_se / lim_near_se` 缩放，并对 `lens_has_past_infinity()` 增加 `-ENDPOINT_HYST_SE` 滞回消抖。
   - `is_parfocal_own_motion()`、`detect_manual_focus_ring_dir()`、`update_manual_focus_ring_state()` 与 `clear_settled_parfocal_cmd()` 兼容 `b6_flags = 0x00` 镜头在死区止步、`<= 2 * tol_se` 过冲及两步 `hw_moving = 0` 收敛期间免误报手动环，同时保留手动拨环打断后的即时 `parfocal_cmd_valid = 0U` 重响应与 `verify_v208 p3` 微步平滑插值门禁。
   - `reset_frozen_window_on_zoom()` 仅在重置或零初值时更新 `dist_last_f01 / dist_last_zoom`，消除长时慢速变焦窗口累积误报；`reset_wake_parfocal_state()` 与 `service_vsync_id04()` 分别保证休眠唤醒重握手零残留及 SPI3 TX-busy 原子回滚。

## 4. V2.23 全闭环验证矩阵（终版 `pub_sha256=7fd7b146...` 全量实测）

| 验证套件 | 命令 / 输出证据 | 最新结果 | 覆盖核心断言 |
|---|---|---|---|
| **V2.23 全时闭环齐焦 4 门禁 + 3 项变异自检** | `verify_v223_fulltime_parfocal.py --pub build/BORYOZA_V223.PUB --selftest` -> `v223_fulltime_parfocal_report.json` | **PASS (`4/4`, selftest=`True`)** | • G1: `17020b1a`、`0x03E3F0`、`189,440 B`、`0` VFP/闭包/64b 除法、`Z8 3.10=0x02230000`、`Z8 3.12=00002302`<br>• G2: `0x807B` 非线性变焦 `1.96m/3.47m/inf`、同帧与分离 `06<->05`、`机身 MF`、`镜身 MF`（含冷启忽略 `0x1D` 零失步超时与近距限幅恢复）全 `<= 1` 分档<br>• G3: 全芯片 `HC32FullChipSim` 空闲 `E0` 防误停、`C0->E0` 制动后重构锚点、`AF-S`/`AF-C`/`机身 MF`/`镜身 MF` 与用户 MF 环打断重锚<br>• G4: `0xC56E` 冷启、远距 `80..300m`、近摄微回退、`lag=2` 与分裂到达、连续变焦+低速 `12/s` 追焦、实机 `Body-MF C0->E0` 制动、`0xC56F 120Hz lag=4` / `0xC13D`、`b6_flags=0` 死区/两步收敛/限位抖动、`0xE0` 滑行期唤醒重握手及 `3` 项变异自检全命中 |
| **V2.22 实机回归 3 门禁 + 3 项变异自检** | `verify_v222_field_fixes.py --pub build/BORYOZA_V223.PUB --selftest` -> `v222_field_fixes_report.json` | **PASS (`3/3`, selftest=`True`)** | `PA0 PFSR=0 / PCR=0` 零倒灌、`0x807B` 与 `0xC56E` 5 轮奇数/偶数次 `POR` 连续唤醒、`0xC56E` 归位全程 `AF` 首报及 `3` 项变异自检全通过 |
| **V2.21 实机回归 3 门禁 + 变异自检** | `verify_v221_field_fixes.py --pub build/BORYOZA_V223.PUB --selftest` -> `v221_field_fixes_report.json` | **PASS (`3/3`, selftest=`True`)** | `CMD 0xD2+0x15` 待机静默、热重启及 `0x807B` 电子齐焦回归全通过 |
| **V2.20 专项 6 门禁 + 变异自检** | `verify_v220_features.py --pub build/BORYOZA_V223.PUB --selftest` -> `v220_features_report.json` | **PASS (`6/6`, selftest=`PASS`)** | `26` 款 3D 光学表、`5` 款适马实时解码、`Ret_SRAM` 无损 `DS13` 热启、`1-LSB` `0x117`、双层熔断与驻车防误唤醒全通过 |
| **V2.19 兼容 8 门禁 + 自检** | `verify_v219_features.py --pub build/BORYOZA_V223.PUB --selftest` -> `v219_features_report.json` | **PASS (`8/8`, selftest=`PASS`)** | `CMD 0xEE` 20fps、电子齐焦、`DS13` `c0`/手动光圈环/TC `0x8138`、`CMD 0x15/16/24` 断链隔离与黑匣子 `v2` 全事件 |
| **V2.18 散焦速度与 MCU 优化** | `verify_v218_defocus_and_mcu_opt.py --pub build/BORYOZA_V223.PUB` -> `v218_defocus_and_mcu_opt_report.json` | **PASS (`5/5`)** | 散焦速度闭环与 MCU 循环优化在 `7fd7b146...` 全通过 |
| **V2.13 实机修复 4 门禁** | `verify_v213_field_fixes.py --pub build/BORYOZA_V223.PUB` -> `v213_field_fixes_report.json` | **PASS (`4/4`)** | `E_SESSION_PASSIVE_MOUNT` 兼容、`CMD 0xF3/10` 时序、光圈与实机重联回归全通过 |
| **V2.12 GM 光学专项 + 自检** | `verify_v212_gm50150_optics.py --pub build/BORYOZA_V223.PUB --selftest` -> `v212_gm50150_optics_report.json` | **PASS (`6/6`, selftest=`PASS`)** | `FE 50-150mm F2 GM` 三维 `CMD 0x24 A/B` 暗角/畸变/LCA 张量与手动光圈环闭环核验 |
| **V2.11 实机修复 4 门禁** | `verify_v211_field_fixes.py --pub build/BORYOZA_V223.PUB` -> `v211_field_fixes_report.json` | **PASS (`4/4`)** | Z8 PDAF 有理解算器双向对称与 `4m<->2m/MFD` 闭环、待机换镜零死锁及 `+40 ms` `UVLO` D2/D3 实拍 |
| **V2.08 硬件真值 16 项** | `verify_v208_hardware_truth.py --pub build/BORYOZA_V223.PUB` -> `v208_hardware_truth_report.json` | **PASS (`16/16`)** | 原厂 BL 烧录、`26` 组 3D 光学表 + `57` 光谱、`e=+2` 实时 `ID05`、`0x807B` 4 向闭环、`0x117` 平滑插值与 `0xC56E` 低电归位/OIS D3 |
| **AF 反馈收敛行为 + 负例自检** | `test_af_feedback_behavior.py --selftest` -> `af_feedback_behavior.json` | **PASS (`23/23`)** | 有限速度执行器多延迟收敛、三角追踪、AF-ON 释放重触发、DMF 退出及 6 项负例自检 |
| **M0/M2 全芯片仿真 + 原厂 BL** | `test_m0_bl_and_mcu_sim.py` -> `m0_m2_full_chip_report.json` | **PASS (`78/78`)** | 原厂 BL 烧录与回读校验、零初始 EFM 写、`SPI1` 热线、热插拔、`0x807B`/`0xC56E` 全流程与 `12` 类黑匣子事件 |
| **MCU 硬件压力 19 项** | `test_stress_hardware.py --selftest` -> `stress_hardware.json` | **PASS (`19/19`)** | 冷启动 `POR 0..300 ms`、重复驻车唤醒、`wake_with_passive_replacement`、热插拔抖动、`SlowStart` 与坏头超时恢复 |
| **M1 主机 ASan/UBSan + Z8 闭环** | `test_m1_core_z8_closed_loop.py --selftest` -> `m1_core_z8_closed_loop.json` | **PASS (`53/53`)** | `ExifTool` 真实 `1.00 m` 回读、`120 Hz` 与 `1.5 Mbps`、`995m` 限位窗、`8` 区温漂与 Z8 `00002302` 版本字 |
| **四品牌跨模型特性链** | `test_brand_feature_chains.py --selftest` -> `brand_feature_chains.json` | **PARTIAL (`565/565`, 0 失败)** | 覆盖 `18` 款实机抓包模型全部 `565` 条断言零失败；实机参数待补条目按模型清单归档于 `brand_feature_chains.json` |
| **全协议矩阵压力测试** | `test_stress_protocol_matrix.py --selftest` -> `stress_protocol_matrix.json` | **PASS (`47,577/47,577`)** | `matrix (18,043)`、`negative (3,720)`、`prehandshake (13)`、`targets (40)`、`descriptors (25,704)`、`m1 (53)` 与 `selftest (4)` |
| **T0/T1/T3 静态、新鲜度与对抗审查** | `check_source_comments.py --selftest`、`flash_usb_bor.py --selftest`、`syntax_check.py`、`lint_check.py`、`qa_oracles.py freshness` | **PASS (`24/24` 注释, `V2.23/189,440B` 刷机自检, `40/40` freshness, 0 lint 失败)** | 零源码注释违规、零 `64-bit` 软件除法、全改动 `clang`/`clang-tidy`/`py-compile` 零失败；Break-It 四轮审签通过（`0 P0 / 0 P1 / 0 P2`） |
