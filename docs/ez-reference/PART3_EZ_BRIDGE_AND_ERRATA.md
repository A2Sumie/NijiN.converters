# Part 3：E-to-Z 协议桥接架构、`V2.16` 控制方程、Known Bad 故障目录与历史报告机器码勘误

本文件为 `BORYOZA SE-NZ`（E-mount 镜头至 Nikon Z 机身适配环固件）第三部分技术参考，覆盖：
1. **E-to-Z 坐标变换、光学灵敏度方程与 `V2.16` 桥接固件架构**（基于 `V2.16` 源码与 `BORYOZA_V200.elf` 机器码反汇编）；
2. **全谱系 `Known Bad` 故障工程目录**（涵盖 `e2nz-fw`、`ETZ21 / Pro / Pro Plus`、`BORYOZA V2.00..V2.16`、Sony GM XD / Sigma / Tamron / Zeiss 镜头、`Z8 3.12` 超时、FAT12 烧录与串口带宽门限）；
3. **经机器码证实的历史文档勘误总汇**（逐条对照 `Nikon Z8 3.12`、`Nikkor Z 50mm f/1.2 S`、`Sony A1 II`、`Sigma OS2` 及 `V2.16` ELF，纠正 `evidence/`、`docs/` 与设计文档中的错误结论）。

工作根目录：[`se-nz/`](../../se-nz/)

---

## 0. 基线产物与实测验证门禁（Baseline Artifacts & Verification State）

### 0.1 `V2.16` 发布固件与段布局边界

| 产物 / 段 | 路径 / 地址范围 | 大小 / 对齐 | 校验 / 关键属性 |
| :--- | :--- | :--- | :--- |
| 发布镜像 `BORYOZA_V216.PUB` | `BORYOZA_V216.PUB` | `157,184 B`（`307 × 512 B` 扇区） | 16 B 帧头 `"PRODUCTVER:1.001"`；SHA-256 `6498d81ae6384927e33895910712c73b39d333be3e0a278d1fadecc7c4feae93`；`program_end_hex = 0x000365F0 <= 0x00060000` |
| 主 ELF `BORYOZA_V200.elf` | `BORYOZA_V200.elf` | `751,980 B` | SHA-256 `de1ebd8c983d7c7ecba65f90b69f8704ee11220e415ea3716ee9f8c70b937e31`；`initial_msp = 0x1FFFFF00`；`Reset_Handler = 0x00010281` |
| 裸二进制 `BORYOZA_V200.bin` | `BORYOZA_V200.bin` | `156,888 B` | SHA-256 `9161fea93eb3baadbd12281de35812d915fdde1d99468e0e42a6b024d5636dea` |
| `.isr_vector` | `0x10000..0x11000` | `4 KiB` | 紧接 `64 KiB` Bootloader（`0x00000..0x0FFFF`） |
| `.readme_ver` | `0x11000..0x11004` | `4 B` | 字节流 `10 02 0b 1a`（`readme_ver_hex = 10020b1a`，对应机身菜单 `LF 2.16`） |
| `.text` | `0x11008..0x35cfc` | `0x24cf4 B` | FPU/闭包门禁（`pack_manifest.json`）：`nofpu_checked_functions_count = 85`（0 违规）、`ram_isr_closure_functions = 32`、`efm_sram_closure_functions = 5`、`closure_violations = 0` |
| `.ramfunc` / `.dma_sramh` | `0x1fff8400`（`0x7cc` B）/ `0x1fffa800`（`0x2664` B） | SRAMH | 高频 ISR 与 DMA 缓冲区 |
| `.optics_sram2` / `.diag_sram3` / `.retram_noload` | `0x20010000`（`0xf848` B）/ `0x20020000`（`0x4000` B）/ `0x200f0000`（`0x1c` B） | SRAM2/3/RetRAM | 光学快照与热复位诊断结构 |

关联固件工程核心源码：[boryoza_types.h](../../se-nz/include/boryoza_types.h)、[startup_hc32f460.S](../../se-nz/src/00_hal/startup_hc32f460.S)、[main.c](../../se-nz/src/main.c)、[optics_xlate_fpu.c](../../se-nz/src/02_core/optics_xlate_fpu.c)、[aperture_controls.c](../../se-nz/src/02_core/aperture_controls.c)、[vr_duty_controller.c](../../se-nz/src/02_core/vr_duty_controller.c)、[Makefile](../../se-nz/Makefile)、[pack_pub.py](../../se-nz/tools/pack_pub.py)、`HANDOFF-2026-11-17-EOD.md`。

### 0.2 `V2.16` 构建产物的本轮 SIL 与自检执行台账

所有下列测试均基于当前原样源码与现行 `build/BORYOZA_V216.PUB` 执行：

| 验证脚本 | 目标 / 范围 | 结果 | 备注 / 已知陈旧用例归因 |
| :--- | :--- | :--- | :--- |
| `verify_v216_gm50150_close_lock_and_lf_ver.py` | GM 50-150 近摄锁定与 `LF 2.16` 同步 | **PASS 4/4** | 验证 `(1+beta)^2` 阻尼、`C0 mosi[11] & 1` 与 `CMD 0x10` BCD 版本号 |
| `verify_v215_z8_closed_loop_large_defocus.py` | Z8 大离焦闭环与限位收敛 | **PASS 5/5** | 覆盖 Sony XD 大行程直达与两段渐近减速 |
| `verify_v214_native_af_and_offaxis.py` | 原生 AF 握手与离轴像差 | **PASS 5/5** | `--pub build/BORYOZA_V216.PUB` |
| `verify_v213_field_fixes.py` | `V2.13` 实拍回归门禁 | **PASS 4/4** | `--pub build/BORYOZA_V216.PUB` |
| `verify_v212_gm50150_optics.py` | GM 50-150 光学数据与 3 平面光瞳 | **PASS 6/6** | `--selftest` |
| `verify_v211_field_fixes.py` | `V2.11` 实拍回归门禁 | **PASS 4/4** | `--pub build/BORYOZA_V216.PUB` |
| `verify_v210_field_fixes.py` | `V2.10` 实拍回归门禁 | **PASS 3/3** | `--pub build/BORYOZA_V216.PUB` |
| `verify_v209_field_fixes.py` | `V2.09` 实拍回归门禁 | **PASS 6/6** | `--pub build/BORYOZA_V216.PUB` |
| `verify_v208_hardware_truth.py` | `V2.08` 硬件真值矩阵 | **PASS 16/16** | `--pub build/BORYOZA_V216.PUB` |
| `test_m1_core_z8_closed_loop.py` | M1 核与 Z8 闭环模拟器 | **PASS 53/53** | `--selftest` |
| `test_m0_bl_and_mcu_sim.py` | Bootloader 与 MCU 外设仿真 | **PASS 78/78** | 全量通过 |
| `test_stress_hardware.py` | 硬件时序与异常压测 | **PASS 19/19** | 含 `cold_por_0ms..300ms`、`park_wake_*`、`repeat_park_*`、`hotplug_bounce_*`、`sony_busy_stalled_encoder_bounded_stop`、`sony_delayed_slow_absolute_start_to_arrival`、`bad_header_large_windows_phase2_timeout`、`phase2_timeout_uint32_wrap` |
| `qa_oracles.py freshness` | 产物新鲜度门禁 | **PASS 2/2** | ELF/BIN/PUB 与源码时间戳一致 |
| `test_stress_protocol_matrix.py` | 遗产 V2.07 协议全矩阵压测 | **FAIL 45,875/47,579**（exit 1） | `descriptors 25,704/25,704 PASS`、`m1 53/53 PASS`、`selftest 4/4 PASS`；FAIL 全部源于 V2.07 陈旧断言：`negative`（`2220/3721`）与 `prehandshake`（`13/14`）因 `V2.16` 合法接收 `13 B` 最短 `ID06`（`short-id06/13 = True` 修改快照导致后续 1500 个坏校验用例与旧基线漂移）；`targets`（`30/40`）因向 `bounds/*/{E2,E4}` 注入 `-2147483648`（`0x80000000` 被当成有限位哨兵）；`matrix`（`17851/18043`）因 `0/<lens>/mf/0/*` `E0` 到位后的 `hot[6]&1` 判定基准陈旧 |
| [flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) | `--selftest` | **FAIL**（exit 1） | 第 240 行硬编码 `assert expected_ver == "V2.15"`，与当前 `"V2.16"` 冲突 |

---

## 1. E-to-Z 转换方程与 `V2.16` 桥接固件架构

### 1.1 坐标映射、光学灵敏度 `K_Z` 与 `(1 + beta)^2` 近摄阻尼

1. **坐标基准与缩放（[af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c)）**
   - E 端以线性位置编码器单位 `S_E` 表征镜组位置，机械行程边界为 `[mech_lz, mech_hz]`。
   - Z 机身以步距脉冲 `C_Z`（`curr_pos_cz` / `delta_cz`）表征对焦行程，局部微分雅可比矩阵为：
     $$\mathrm{jacobian} = \left|\frac{\mathrm{d}C_Z}{\mathrm{d}S_E}\right|$$
     由镜头配置表 `[r3 + 0x2c]` 给出。

2. **Sony 机身预乘光圈因子解耦（`calibrated_emount_k` @ `0x15ee6..0x15f7e` 与 `af_servo_refresh_kz` @ `0x15edc..0x15fe0`）**
   - 在 Sony `A1 II`（ROM `0xAA2A32`，验证文件 `sony_a1ii_k_div.txt`）上，机身通过下式将 E 卡口 `ID06.defocus_k_se`（`raw_k`）换算为行程脉冲：
     $$\Delta S_E = \mathrm{round}\!\left(\frac{32 \cdot d_{\mu\mathrm{m}} \cdot \mathrm{raw\_k}}{\mathrm{round}(384 \cdot F_{\mathrm{cur}})}\right)$$
     可见 E 卡口镜头上报的 `raw_k` 已被预乘以与当前光圈成正比的因子 $12 \cdot F_{\mathrm{cur}}$。
   - Nikon Z 机身接收的 `K_Z` 为**与光圈无关**的焦面单位灵敏度（每单位像面失焦量对应的 Z 步距脉冲）。`V2.16` 在 `calibrated_emount_k` 中先从 `ID06` 剥离 E 端光圈预乘因子、还原纯焦面敏感度后乘以 `jacobian`，对于 Sigma `0xC55A` 再乘 `0.5F`（`0x15f82..0x15fa6`）。

3. **近摄放大率 `(1 + \beta)^2` 无条件衰减（`0x15f8a..0x15fba`）**
   - 依据薄透镜高斯像方灵敏度缩放关系 $\frac{\mathrm{d}x'}{\mathrm{d}p} \propto \frac{1}{(1 + \beta)^2}$，`V2.16` 在 `af_servo_refresh_kz` 对 `calibrated_emount_k * jacobian` **无条件**除以 $(1.0 + \beta)^2$（对 Sony XD 与 Sigma 统一生效）：
     $$K_{Z,\mathrm{eff}} = \frac{K_{Z,\mathrm{inf}}}{(1.0 + \beta)^2}$$
   - 对 `Sony FE 50-150mm F2 GM`（`0x807B`）实测：在近摄 $\beta = 0.16$ 时，$(1 + 0.16)^2 = 1.3456$；`V2.16` 输出由无穷远 `kz_inf = 6,897,664` 降至近摄 `kz_close = 5,128,192`（实测比值 `0.7435`），消除 +34.54% 的近摄过冲，残余误差收敛至 `3 SE`。

4. **`K_Z` 对偶编码与非线性禁用（`af_servo_encode_kz_pair` @ `0x159c8` 与 `z_snapshot_builder.c`）**
   - `af_servo_encode_kz_pair` 将浮点灵敏度编码为 `mant | (shift << 8)` 格式，其中 `shift` 钳位在 `0..22`，最小合法编码为 `0x0100`。
   - `z_snapshot_publish` 在 `CMD 20[0x63..0x6C]` 两个非线性曲线槽均写入 `KZ_NONLIN_DISABLED = 0x0100`（双字节组合 `0x01000100`），显式告知 Z8 机身关闭二次/三次多项式非线性修正（Z8 `0x40DB5200..0x40DB5310` 置 `c2 = c3 = 0`）。

### 1.2 有符号 7-bit 指数 Z 速度编解码方程

Z8 机身速度生成器（`0x40DEE9D8`）与固件解码器 `af_servo_decode_z_speed_pps`（`0x15a74`）严格互逆：

1. **特殊码字**：
   - `(spd_lo, spd_hi) == (0x00, 0x00)` → `vpps = 0.0F`（静止/零速）；
   - `(spd_lo, spd_hi) == (0xFF, 0x3F)` → `unlimited = true, dir = +1`（无限速直达）。
2. **一般速度码字解码公式**：
   - 方向位（`spd_hi.bit0`）：
     $$\mathrm{dir} = (\mathrm{spd\_hi} \mathbin{\&} 1) \neq 0 \;?\; -1 : +1$$
   - 有符号 7-bit 指数提取：将 `spd_hi` 最低位清零后按有符号 `int8_t` 算数右移 1 位并钳位至 `[-7, 24]`：
     $$\mathrm{even\_s8} = (\mathrm{int8\_t})(\mathrm{spd\_hi} \mathbin{\&} \mathrm{0xFE}), \qquad \mathrm{exp7} = \mathrm{clamp\_s32}\!\left(\frac{\mathrm{even\_s8}}{2},\,-7,\,24\right)$$
   - 脉冲速度（pps）：
     $$v_{\mathrm{pps}} = (256.0 + \mathrm{spd\_lo}) \cdot 2^{\mathrm{exp7} - 9}$$
3. **慢速微动验证样例**：当 `(spd_lo, spd_hi) = (0x90, 0xFA)` 时，`even_s8 = (int8_t)0xFA = -6`，`exp7 = -3`，解码得 $(256 + 144) \cdot 2^{-12} = 400 / 4096 = 0.09765625\;\mathrm{pps}$（负向微步，用于 F1.2–F2.0 慢速对焦搜索）。

### 1.3 `CMD 0x1D/0x00` vs `(0x1D, 0x3C)` 调度规则与两段渐近减速

1. **峰值速度表 `lens_max_velocity_tps`（`0x15934..0x159c4`）**
   - `is_fast_sony_xd(ctx)`（`brand == 1 && (hw_fam2_ext != 0 || motor_family == 2)`）：返回 `SONY_XD_VMAX_SE_TPS = 150000.0F`（IEEE-754 编码 `0x48127c00`）。
   - Sigma `0xC56E / 0xC55F`：调用 `sigma_150600_thermal_vmax_tps(temp_c)` 返回随温度衰减的速度上限。
   - 其他镜头：返回 `60000.0F`（`0x476a6000`）。
   - 双浮动对焦附加阻尼：若 `is_dual_floating && beta > 0.25F`，对上述结果乘以 `0.8F`（`0x3f4ccccd`）。

2. **单步最大跨度 `compute_1d_max_span_step`（`0x151dc..0x15244`）**
   - 计算绝对机械行程 `abs_span = |mech_hz - mech_lz|`（当 `< 16` 时回退为 `16384`）：
     - Sigma 连续跟踪（`brand == 2 && is_continuous_track != 0`）：`abs_span / 24`；
     - 快速 Sony XD（`is_fast_sony_xd`）：`abs_span / 4`；
     - 常规镜头：`abs_span / 8`。
   - 返回值下限钳位为 `MIN_1D_STEP_SE = 16`。

3. **直达判据 `can_single_shot_direct_abs`（`0x169c6..0x16a20`）与 `decide_singleshot_1d`**
   - 若 `e_lens_id == 7` 或 `is_continuous_track != 0`，恒禁用单次绝对直达 `CMD 0x1D/0x00`。
   - 否则满足以下任一条件即发射 `CMD 0x1D/0x00`（直达绝对位置），不满足则走 `(0x1D, 0x3C)` 相对步进：
     1. `unlimited_speed != 0`；
     2. `fam2_ext != 0` 且 `cmd_vel_se_tps >= 65536.0F || cmd_vel_se_tps >= lens_max_velocity_tps(ctx)`（`0x169e6..0x16a1c`）；
     3. 经 `eval_fam2_approach` 判定当前已在单帧可达或捕获容差窗口内。

4. **两段式内区间渐近减速 `eval_fam2_approach`（`0x15248..0x152f8`）**
   - 定义 `min_step = is_continuous_track ? 10 : 16`。
   - **提前直达收口**：若 `fam2_ext && hw_moving && (|pred_err| <= step_vd || sign_crossed)`，函数返回 `true`，促使调度器提前转入 `0x1D/0x00` 精准落位，防止以高速步进冲过对焦点。
   - **内区间两段渐近减速（`fam2_ext && mech_lz < target_se < mech_hz`）**：
     - 当剩余预测误差 `|pred_err| < 2 * step_vd`：将步长缩减为 `clamp(|pred_err| / 2, min_step, step_vd)`；
     - 当剩余预测误差 `2 * step_vd <= |pred_err| < 3 * step_vd`：将步长缩减为 `clamp((|pred_err| * 3) / 4, min_step, step_vd)`；
     - 其核心目的在于让高速 XD 线性马达在到达机械端止或目标峰前平滑卸荷，兼顾大离焦收敛速度与细搜索零震荡。

### 1.4 Z 机身 AF 指令处理规则（`0x17170..0x17748`）

| 指令 | 刹车 / 中止条件 | 速度与目标解析逻辑 | 适配边界 / 特殊分支 |
| :--- | :--- | :--- | :--- |
| **`CMD 0xC0`** | `(mosi[11] & 0x06) != 0` | `mosi[11].bit0` 在 Z 机身仅选择 `CMD 0x20` 静态/录像限速表，对 `raw_1` 有效性与否无开关语义。<br>• `raw_1 == 0`：调用 `resolve_sentinel_target(raw_13)`（`0x80000000` 解析至 `mech_lz / mech_hz`）；<br>• `raw_1 != 0`：`rel = cur_cz + raw_1`，当 `raw_13 != 0 && raw_13 != 0x80000000` 时启用**同向限位壁钳位**：若 `raw_1 > 0 && raw_13 >= cur_cz` 取 `min(rel, raw_13)`；若 `raw_1 < 0 && raw_13 <= cur_cz` 取 `max(rel, raw_13)`；其余（对侧值或 `0xFFFFFFFF`）安全忽略 | 消除旧版将 `0xFFFFFFFF` 或 `mosi[11].bit0` 误判导致近距峰值回退或极限推离 |
| **`CMD 0xE4`** | `(mosi[9] & 0x06) != 0` | `unused_spd1 = (mosi[7] == 0 && mosi[8] == 0) || (mosi[7] == 0xFF && (mosi[8] & 0x7E) == 0x3E)`。<br>• 若 `(delta_cz == 0) && !(mosi[9] == 0 && !unused_spd1) && (mosi[5] != 0 || mosi[6] != 0)`：取双速度最小值 `min(vpps0, vpps1)`（[af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c)）；<br>• `mosi[9] != 0`：调用 `resolve_native_e4_wall`；<br>• `mosi[9] == 0 && delta_cz != 0`：相对位移 + 同向限位壁钳位 | 与 Z8 `0x40DD2EE0..0x40DD2F80` 完全对齐 |
| **`CMD 0xE2 / 0xE6`** | **仅当** `(mosi[7] & 0x06) == 0x06` | Bit 1 与 Bit 2 仅在**同时置位**（`0x06`）时急停中止；单 bit 置位属正常移动与方向标。`fast_direct = unlim || (is_fast_sony_xd && vpps >= FAST_JUMP_CZ_VPPS)` | `E2/E6` 与 `C0/E4` 刹车掩码不可混用 |
| **`CMD 0xEE`** | 无急停掩码 | 置位 `is_continuous_track = 1`，绝对目标位置取 `mosi[1..4]` | 切换至连续追踪（AF-C / 录像跟随）模式 |
| **`CMD 0xEC`** | 无急停掩码 | 清零 `is_continuous_track = 0`，无限速绝对移动至 `mosi[1..4]` | 镜头重置/高速回位专用直达指令 |

### 1.5 `CMD 20` 快照、`CMD 0x10` 固件版本同步与猫眼光瞳配置

在 `z_snapshot_publish`（`0x1b2d0..0x1b97c`，源码 [z_snapshot_builder.c](../../se-nz/src/02_core/z_snapshot_builder.c)）中：

1. **`CMD 0x10` BCD 固件版本联动**：
   - `cmd10[0x03..0x06]` 写入 `0x02160000`（小端布局中 `cmd10[0x05] = 0x16`、`cmd10[0x06] = 0x02`），Z8 读取属性 `0x005`（`[obj_0xb, 0x0a..0x0b]`）后在机身版本菜单中显示 **`LF 2.16`**，与链接段 `.readme_ver @ 0x11000`（`10 02 0b 1a`）及 USB `README.TXT` 严格一致。
   - `cmd10[0x13..0x16]` 固定为 `0x1a1a2a25`。

2. **`CMD 20` 静态 / 录像速度上限与 odd-bit 约束**：
   - `cmd20[0x85..0x88]`（Z 属性 `0x102`，`max_vpps_stills`）：
     - `is_fast_sony_xd`（`0x1b666 -> 0x1b926 -> 0x1b68c`）：写入 **`0x00040000`**（`262,144 pps`，SPI 字节流 `00 00 04 00`）；
     - 其余镜头（含 Sigma `0xC56E`）：写入 **`0x00010000`**（`65,536 pps`，SPI 字节流 `00 00 01 00`）。
     - 二者均严格保证 **`(cmd20[0x86] & 1) == 0`**，规避 Z8 速度解析器在 `byte1.bit0 == 1` 时的异常截断分支。
   - `cmd20[0x89..0x8C]`（Z 属性 `0x103`，`max_vpps_movie`）：写入 `0x00004000`（`16,384 pps`）。
   - `cmd20[0x8D]`（Z 属性 `0x0FF`）：反映对焦限位器（Focus Limiter）开关状态；`cmd20[0xB9]` 写为 `barrel_limiter_on ? 170 : 255`。

3. **3 平面猫眼光瞳模型（`write_cmd20_pupil_block` @ `0x1b8f4..0x1b950`）**：
   - 当同时满足 `brand == 1`（Sony）、公称焦距 `f_nom >= 85 mm` 且最大光圈 `fnum_open <= F3.5`（编码值 `<= 896`）时，启用 3 平面圆孔猫眼光瞳描述（`Plane 2 + Plane 3`，`h3 = h4 = 0`），优化中长焦大光圈边缘虚化与机身遮光/防抖几何计算。

### 1.6 限位缓冲锁存、Sigma `0xC56E` 行为保留与 VSync / UART 时隙约束

1. **端止缓冲与卡滞超时停机（`check_endstop_cushion_hit` @ `0x1511c..0x151da` 与 `af_servo_build_id04_subrecord`）**：
   - 缓冲带宽度：`band = clamp_s32(|mech_hz - mech_lz| / 128, 24, ...)`。
   - 当马达进入端止缓冲带并发生位置停滞/未到目标时，失速判定门限按实时帧率切换：
     $$\mathrm{stall\_limit} = (\mathrm{active\_vsync\_hz} \ge 120) \;?\; 28 : 14$$
   - 计数到达 `stall_limit` 时置位 `stalled_abort = arrival_confirmed ? 0 : 48`，立即在下一周期 `ID04` 下发 `0x1C` 刹车停机，阻止马达在机械硬限位上持续顶死或拖垮 Z 机身 AF 超时计数器。

2. **Sigma `0xC56E`（150-600）兼容性保留原则**：
   - 保留 `mech_lz = -512`（无穷远端越程余量）；
   - 保持 `0x1D` 步进式绝对/相对驱动、`CMD 20` 静态上限 `0x00010000`（`65,536 pps`）及热补偿上限 `sigma_150600_thermal_vmax_tps(temp_c)`；
   - `120 Hz` 高刷严格门控于 `lens_session.c`（[lens_session.c](../../se-nz/src/02_core/lens_session.c)，`0x132d2..0x132de`）：
     $$\mathrm{allow\_120} = (\mathrm{uart\_baud} \ge 1500000) \mathbin{\&\&} (\mathrm{brand} == \mathrm{LENS\_BRAND\_SIGMA}) \mathbin{\&\&} (\mathrm{supports\_120hz} \neq 0)$$
     机器码对应 `movne r1, #120; moveq r1, #60`。

3. **Sony `0x807B`（50-150 GM）`60 Hz` VSync 与 18 组 UART 带宽门限**：
   - Sony 镜头恒运行在 `60 Hz`（`TMRA4_PERAR_60HZ = 49999`，周期 `16.667 ms`，`cmd0d = 0x00`）。
   - `e_usart1_dma.c`（[e_usart1_dma.c](../../se-nz/src/01_transport/e_usart1_dma.c)，`0x12960..0x1299c`）异步槽位边界（`3 MHz` 定时器刻度）：
     - `active_vsync_hz > 119`：`[18600, 22800]` ticks（`6.200..7.600 ms`）；
     - `60 Hz` 分支：`[33600, 45600]` ticks（`11.200..15.200 ms`），`session_state == 20` 时上限钳位为 `<= 45499` ticks。
   - **18 组 `ID05` 串口时间账本**（8N1 = `10 bit/B`，`GRP05_LEN[18] = {8, 1, 8, 3, 10, 28, 2, 6, 1, 2, 8, 6, 6, 6, 1, 6, 3, 3}`）：
     - 15 组载荷 `96 B`（线上封装帧 `105 B`）；18 组载荷 `108 B`（线上封装帧 `117 B`）；
     - 在 `1.5 Mbps`（`6.667 us/B`）下仅收发基础帧即需：`ID05(117 B) = 780.0 us`、`ID06(25 B) = 166.7 us`、`ID03(29 B) = 193.3 us`、`ID04(27 B) = 180.0 us`（`750 kbps` 时翻倍至 `1560.0 us + 333.3 us + ...`），叠加 `PA2` 握手建立 `20 us` + 保持 `90 us` + 镜头侧响应翻转延迟 + 异步突发对焦指令，在 `120 Hz`（`8.333 ms`）下剩余时隙低于高并发 AF 突发需求，故 Sony 镜头强制锁定 `60 Hz`。

### 1.7 HC32F460 Bootloader 边界与 USB `FAT12 BOR` 升级约束

根据 Bootloader（`0x00000..0x0FFFF`）实测机制（参考 [USB-LIFECYCLE.md](BOOTLOADER_AND_USB_MSC_CONTRACT.md)）：

| 区域 / 参数 | 地址 / 规格 | 硬件 / 烧录器约束 |
| :--- | :--- | :--- |
| Bootloader 代码与只读常量 | `0x00000..0x0FFFF`（`64 KiB`） | `ICG @ 0x400..0x41F`；`UID @ 0x41F0..0x420F`；USB 枚举 `VID=2E88, PID=4601` |
| 虚拟 FAT12 磁盘布局 | `906 × 512 B` 扇区，卷标 `"BOR     "` | `1 sec/cluster`、`1 reserved`、`1 FAT`（4 扇区）、根目录 16 项在 `LBA 5`、`README.TXT` 占 cluster 2 `LBA 6..9`（由 `0x4F58` 读取 APP `0x11000` 格式化）；数据区起于 `LBA 10..905` |
| 主应用程序区上限 | `0x10000..0x5FFFF`（`<= 320 KiB`） | 编译输出 `program_end_hex` 严禁越过 `0x60000` |
| 诊断 / 日志扇区连带擦除 | `0x7A000..0x7FFFF` | `0x5070` 检测到首 16 B 为 `"PRODUCTVER:1.001"` 后会**整片擦除 `56 × 8 KiB` 扇区（`0x10000..0x7FFFF`）**，即每次 USB 升级都会清空全部黑匣子与历史故障记录 |
| 连续 LBA 与跨界红线 | `LBA >= 10` 连续簇写 | `phase 2` 遇到任何非连续 LBA 跳跃即静默 `return 0`（不完整烧录或变砖）；向 `LBA < 10` 跨界写（如 `LBA 9, count 2`）会直接覆盖 `0x9A7C` 附近 IRQ 回调向量；macOS Finder 产生的 `._*` AppleDouble 与 Spotlight 写操作会打断连续扇区，必须使用 [flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) 顺序直写 |

---

## 2. 全谱系 `Known Bad` 故障工程目录

| 编号 | 影响对象 / 版本 | 故障现象与触发条件 | 根因分析 | 修复 / 规避边界 |
| :--- | :--- | :--- | :--- | :--- |
| **KB-01** | `e2nz-fw` `V1.27..V1.35` | SIL 仿真通过但实装 Nikon Z8 握手失败报 `DEVICE_FAIL` | 反编译迁移补丁在实机 SPI/初始化时序与状态字不自洽（`V1.20` `48,060 B` PUB 为最后一个稳定基准版） | 已全面退役；由干净室 C17 `se-nz` `V2.00..V2.16` 替代 |
| **KB-02** | `BORYOZA V2.00..V2.07`（含 `AD91`） | Z8 `CMD 0x0D` 与实际 VD 帧率不匹配；合焦后 Z8 仍频繁重新扫焦或焦点恢复报错 | `AD91` 上报 `0D 01`（标称 50 Hz）却以 60 Hz 驱动 VD；`HOT[6]` 稳定态 bit[5:4] 残留非零（`0x36/0x26`），触发 Z8 `410789E0..41078B94` 焦点恢复消费者报错（`1` 而非 `0`） | `V2.16` 对 60 Hz 镜头上报 `cmd0d = 00`，且严格对齐 `HOT[6]` 稳定态标志（`bits[5:4] == 0`） |
| **KB-03** | `0x807B`（Sony 50-150 GM）+ Z8 `3.12` | 大光圈 F2 近摄小物体 CDAF 峰值回退失效，表现为向相反方向“推离”焦点（push-back） | 错误将 `CMD 0xC0 mosi[11] & 1` 解释为“忽略 `raw_1`”，丢弃 Z8 下发的微调反向步数 | 正确解析 `mosi[11] & 1` 为静态/录像限速选择位，始终执行 `raw_1` 与同向 `raw_13` 限位壁钳位 |
| **KB-04** | `0x807B` 及非越无限位 E 镜头 + Z8 `3.12` | 无穷远对焦拉风箱后在 Z8 `0x40DD0B40` 触发 2.0 s 对焦超时 | 沿用 Sigma `-512` 越限值作为全局 `L_Z`，导致真实端止 `cur_cz = 0` 永远无法满足 Z8 `0x40DF0838` 的 `cur_cz <= L_Z` 判据 | 按镜头家族精确建表：`0x807B` 取 `mech_lz = 0`；`0xC56E` 保留 `mech_lz = -512` |
| **KB-05** | `0x807B` 近摄（`beta > 0`） | 近摄合焦过程严重过冲（`beta = 0.16` 时 +34.54%）并反复震荡 | 错误以为“Sony 机身端已补偿 `(1+beta)^2`”而省略适配环侧衰减，但 Z8 因 `KZ_NONLIN_DISABLED = 0x0100` 强制采用线性脉冲换算 | `V2.16` `af_servo_refresh_kz` 无条件除以 `(1.0 + beta)^2`，近摄误差收敛至 `3 SE` |
| **KB-06** | `0x807B` 运行 `120 Hz` VSync 尝试 | 120 Hz 高并发对焦时串口帧撕裂、丢包或卡死 | 18 组 `ID05`（`117 B`）+ `ID06/03/04` + 握手保持时间在 `8.333 ms` 内严重挤压异步 AF 突发带宽 | `V2.16` `lens_session.c` 严格限制 `120 Hz` 仅对 Sigma 生效，`0x807B` 锁定 `60 Hz`（`PERAR = 49999`） |
| **KB-07** | `CMD 20[0x86]` odd bit 与 `0x00040000` 误配 | 高速 XD 镜头单步限速异常、AF-S 长距离拉多次步进才合焦 | `CMD 20[0x85..0x88]` 若令 `(c20[0x86] & 1) == 1` 会触及 Z8 速度解析奇数位缺陷；上限未按 XD 设为 `262,144` 则 Z8 限速压死单步速度 | `is_fast_sony_xd` 写入 `0x00040000`（`00 00 04 00`），常规镜头写入 `0x00010000`，均严守 `(c20[0x86] & 1) == 0` |
| **KB-08** | `CMD 0xE2 / 0xE6` 刹车误判 | Z8 正常发出的追踪/方向标志位 `0x02` 或 `0x04` 被当成急停中止，对焦半途卡死 | 将 `E2/E6` 的 `mosi[7]` 错用 `C0/E4` 的 `(flags & 0x06) != 0` 掩码 | 依据 `50/1.2 S` `0xFFF2721B..0xFFF2723D` 修改为仅 `(mosi[7] & 0x06) == 0x06` 时急停 |
| **KB-09** | `CMD 0xE4` `delta_cz == 0` 且双速同时下发 | 原地切换限速或慢搜切换高速时沿用无效速度引发超时/过冲 | 缺少 Z8 `0x40DD2EE0..0x40DD2F80` 的双速度取小协议分支 | `V2.16` 在 `delta_cz == 0` 分支显式执行 `min(vpps0, vpps1)` |
| **KB-10** | `CMD 0x10` 版本号写死与误指 `0xF3` | Z8 菜单永远显示 `LF 1.00`，无法区分现场实装版本 | 将 `CMD 0x10[3..6]` 硬编码为 `00 01 00 01`，或误以为版本号来自 `CMD 0xF3` | `V2.16` `z_snapshot_publish` 直接同步 `CMD 0x10[0x05..0x06] = 0x16, 0x02`（`LF 2.16`） |
| **KB-11** | `Sigma 0xC56E`（150-600）与 `0xC55A / 0xC55F / 0xC583` | `0xC56E` 高温失步或无穷远余量丢失；`0xC55A` 增益加倍震荡；`0xC583` 报文解帧偏移 | 误改 `0xC56E` `mech_lz = 0` 或取消热表 `sigma_150600_thermal_vmax_tps`；忽略 `0xC55A` `0.5F` 因子；将 `0xC583` `3E` 当 `14B` 而非 `12B` | 保留 `0xC56E` `-512` / `0x1D` / `65536` / 热表；`0xC55A` 乘 `0.5F`；按实机抓包使用 `12B` 净荷（`20B` 线帧） |
| **KB-12** | `Tamron`（`A065/A067/A084/F072/AA056` / `Di III OSD`） | 无防抖 `OSD`（`C133/C134/C135`）被误启用防抖，VR/状态回报异常 | 字符串子串 `" OS"` 误命中 `"OSD"`，错误下发 `CMD 12 = 94 C4` | 改用词界/精确型号区分 OIS 与 OSD，`OSD` 严格下发 `CMD 12 = 00 44` |
| **KB-13** | `Zeiss Touit`（`C031 / C032 / C033`） | 机身几何/暗角/裁切行为错乱 | 历史表将 APS-C DX 镜头误标为全画幅 FX | 纠正为 APS-C DX 标识（`0x10`） |
| **KB-14** | `ETZ21` 历史代际参照风险 | 在不同版本 ETZ21 间直接复用静态反汇编偏移或混淆加解密 | `Ver3.55`（`132,024 B`）/ `Pro V1.37`（`77,750 B`，295 条记录）带静态 per-`(tag, blk)` 加密；明文 `Pro Plus 0204`（`65,320 B`）与 `0207`（`67,912 B`，VA `0x08006000+off`，Cortex-M3，261 条目录 vs 177 条）架构异于 Boryoza HC32F460 | 仅参考其协议语义真值（如 `0x0800BD30` 的 `1D/3C` 与 `HOT[6].bit7 == 0`），严禁将 ETZ21 Cortex-M3 内存布局套入 HC32F460 |
| **KB-15** | macOS Finder / 非连续 USB 写 `BOR` 盘 | 烧录静默中止变砖、覆盖中断表或丢失黑匣子日志 | Finder 写入 `._*` / Spotlight 打断 `phase 2` 连续 LBA（`return 0`）；`LBA < 10` 跨界踩坏 `0x9A7C` 回调；全擦 `0x10000..0x7FFFF` 清空 `0x7A000..0x7FFFF` | 强制使用 [flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) 直写；烧录前备份高区黑匣子；更新 `flash_usb_bor.py:240` 与 `test_stress_protocol_matrix.py` 陈旧版本断言 |

---

## 3. 经机器码证实的历史文档勘误总汇（Machine-Code-Proven Errata）

本节审计 `evidence/`、`docs/` 及关联历史研究笔记中的所有实证错误，每一条均以 `Nikon Z8 3.12`（`sec2_eg2070_031200.bi`，SHA-256 `0a0a0afda5d11049bbbbab698c8db88223fda6ecf2d1231c68aa8f8f93b0a09a`）、`Nikkor Z 50mm f/1.2 S`（`seg0_bank0_fff00000.rx.bin`）、`Sony A1 II`、`Sigma OS2` 及 `BORYOZA V2.16` ELF 机器码为准绳。

### 勘误 (a)：`CMD 0xC0 mosi[11] & 1` 的真实语义（选择器而非 `raw_1` 屏蔽开关）

- **错误报告**：早期逆向笔记本与基于旧推论的适配逻辑认为 `CMD 0xC0 mosi[11] & 1 == 1` 表示“忽略相对偏移 `raw_1`（仅执行辅助控制）”。
- **机器码直接反证**：
  - **Z8 `3.12` 机身侧**：`0x40DCFA8C -> 0x40DCA360 -> 0x40E10268` 将静态 vs 录像/LV 模式标志 `w0 & 1` 独立存入 `[sp, 0x1c]`；目标增量 `raw_1` 在 `0x40DCFBDC` 独立写入 `±w26`（`w26 = (w0 == 0) ? 0x19 : w0`）。随后 `0x40DEEA54` 将 `[x20 + 4]` 复制到 `[x0 + 0x1c]`，由 `0x410A005C` 执行 `bfxil w9, w10, 0, 1` 打包进 `mosi[11].bit0`（交叉验证材料：`z8_final_gap_check_disasm.txt`）。
  - **Nikkor Z `50mm f/1.2 S` 镜头侧**：在 `C0` 处理路径 `0xFFF274B8..0xFFF274E4`（以及 `E4` 路径 `0xFFF276ED..0xFFF27707`）中，镜头固件执行 `btst #0x0, r12; stz r9, r8; mov.L [r8], r9; min.UB r15, r9`，精确根据 `bit0` 在 `[0x22a8 + r6]`（录像/LV 速度上限）与 `[0x22ac + r6]`（静态照片 `CMD 0x20` 速度上限）之间二选一并用于 `min` 限速，**没有任何位置用 `bit0` 屏蔽或覆盖 `raw_1`**（验证记录：`z8_final_3_items.txt`）。
- **协议失真后果与修正**：按旧错误模型丢弃 `raw_1` 会直接破坏 F2 近距离小目标 CDAF 越峰后的微小回退指令，表现为反推离焦。`V2.16` 始终保留并执行 `raw_1`。

### 勘误 (b)：伪造的 `Z8 CMD 0xC0 mosi[11] = 0x2D` 记录

- **错误报告**：[Z8_AND_SONY_A1II_A9III_AF_ARCHITECTURE_STUDY.md](Z8_SONY_AF_ARCHITECTURE_STUDY.md) 第 30 行宣称 Z8 下发的 `CMD 0xC0 mosi[11]` 典型值为 `0x2D`。
- **机器码直接反证**：
  - 在 `Z8 3.12` 函数 `0x40DCDB2C -> 0x40DED9F4`（`0x40DEDA70: stp wzr, wzr, [x26, 0x3c]; stp w24, wzr, [x26, 0x34]`）与伴生入口 `0x40DEDAA0`（`0x40DEDB24: stp wzr, wzr, [x26, 0x3c]; stp w24, wzr, [x26, 0x34]`）中，对应 `mosi[11]` bits `2..1` 及高位扩展槽 `[x26 + 0x38..0x40]` 被指令直接清零（`wzr`）。
  - 因此 `0x410A005C` 在全部非刹车路径下**只能**合成 `{0x00, 0x01, 0x08, 0x09}`（与实测及 `0x40DCFBB8` 路径一致，静态典型值为 `0x01`），Z8 3.12 机身在正常非刹车路径从不生成 `0x2D`（`0x20/0x04` 位在 Z8 AF 路径全为 0）。

### 勘误 (c)：`CMD 0xC0/0xE4` 与 `CMD 0xE2/0xE6` 急停刹车掩码的本质差异

- **错误报告**：早期参考与初步逆向将 Z 卡口全系列长包 AF 指令的急停条件混同为统一的位掩码判定。
- **机器码直接反证**（`Nikkor Z 50mm f/1.2 S` Bank 0 镜像 `seg0_bank0_fff00000.rx.bin`，验证记录 `z8_final_3_items.txt`）：
  - **`CMD 0xC0`**（`0xFFF27408..0xFFF274A7`）：执行 `tst #0x6, r12; bne.W` — 只要 `(flags & 0x06) != 0`（Bit 1 或 Bit 2 任一置位）立即刹车中止。
  - **`CMD 0xE4`**（`0xFFF2762C..0xFFF276AF`）：执行 `tst #0x6, r14; bne.W` — 同为 `(flags & 0x06) != 0` 即刹车中止。
  - **`CMD 0xE2 / 0xE6`**（`0xFFF2721B..0xFFF2723D`）：执行 `and #0x6, r15; cmp #0x6, r15; beq.B` — **必须** `(mosi[7] & 0x06) == 0x06`（Bit 1 与 Bit 2 同时为 1）才触发刹车中止；单置 Bit 1 或 Bit 2 属于正常追焦方向/阶段运行态。
- **协议失真后果与修正**：对 `E2/E6` 误用 `!= 0` 掩码会在 Z8 正常运行期间把合法移动误杀为急停。`V2.16` 分家族精确实现了两套掩码逻辑。

### 勘误 (d)：`CMD 0xC0 raw_13` 同向限位壁与 `CMD 0xE4 delta_cz == 0` 双速取小规则

- **错误报告**：早年笔记将 `CMD 0xC0 raw_13` 仅视为绝对目标补充或恒忽略，并将 `CMD 0xE4` 当作单一速度字段指令（忽略 `mosi[7..8]` 副速度与 `delta_cz == 0` 分支）。
- **机器码直接反证**：
  - **`CMD 0xC0 raw_13`**：在 `50mm f/1.2 S` `0xFFF27571 -> 0xFFF2816E..0xFFF28188` 中，当 `raw_1 != 0` 且 `raw_13` 非零、非 `0xFFFFFFFF`（无壁）、非哨兵（`0x7FFFFFFF` 在 50/1.2 内部映射到 `0x7FFF0000 / 0x80010000`；适配器侧对 `0x80000000` 解算端止）时，`raw_13` 构成**方向性搜索限位壁**，仅截断正向过界 `raw_13 >= cur_cz` 或负向过界 `raw_13 <= cur_cz`。
  - **`CMD 0xE4 delta_cz == 0`**：在 `Z8 3.12` `0x40DD2EE0..0x40DD2F80`（含 `0x40DD2F24` 处乘魔数 `0x24924925` 做除以 7 运算）证实在 `delta_cz == 0` 时机身下发双速度字段 `[x2]`（`vpps0`）与 `[x2 + 4]`（`vpps1`），镜头端以 `min(vpps0, vpps1)` 收敛限速。`V2.16` 在 [af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c) 完全对齐该逻辑。

### 勘误 (e)：近摄 `K_Z` `(1 + beta)^2` 衰减必要性（反证“Sony 镜头无需除倍率”）

- **错误报告**：[Z8_AND_SONY_A1II_A9III_AF_ARCHITECTURE_STUDY.md](Z8_SONY_AF_ARCHITECTURE_STUDY.md) 第 97 行断言 Sony E 卡口镜头无需在桥接层对 `K_Z` 除以 `(1 + beta)^2`。
- **机器码直接反证**：
  - `Z8 3.12` `0x40DB5200..0x40DB5310 -> 0x40DB7F88` 与 `0x40DB76B0` 显示：由于适配器在 `CMD 20[0x63..0x6C]` 上报 `KZ_NONLIN_DISABLED = 0x0100`（显式将 Z 机身多项式高阶系数 `c2 = c3 = 0`），Z8 在整个物距范围内对 Sony 镜头**强制执行线性离焦到步距换算**，并不额外叠加 `(1 + beta)^2` 的光学倍率补偿（交叉验证：`z8_kz_cmd10_f3_check.txt` 与 `z8_deep_verify.txt`）。
  - 实测 `0x807B`（50-150 GM）在 `beta = 0.16` 若省略适配器侧 `(1 + 0.16)^2 = 1.3456` 衰减，闭环步进将固定过冲 **`+34.54%`**；加入无条件 `(1 + beta)^2` 除法后（`kz_inf = 6,897,664 -> kz_close = 5,128,192`），步进误差压低至 `3 SE`。

### 勘误 (f)：机身 `LF` 固件版本号源自 `CMD 0x10[0x05..0x06]`，而非 `CMD 0xF3` 或静态硬编码

- **错误报告**：[BORYOZA_NEW_FW_DESIGN_SPEC.md](HC32F460_SE_NZ_DESIGN_SPEC.md) 第 44、200 行将 `CMD 0x10[3..6]` 硬编码为 `00 01 00 01`（`LF 1.00`）并宣称其与 `.readme_ver @ 0x11000` 相互独立；另有部分早期记录误将机身版本显示归因于 `CMD 0xF3`。
- **机器码直接反证**：
  - `Z8 3.12` `CMD 0x10` 接收处理函数 `0x410A195C` 直接把报文偏移 `[x19, 5..6]`（即 `(minor_bcd, major_bcd)`）写入 `[obj_0xb, 0x0a..0x0b]`（对应属性 `0x005`，经 `0x4109EF0C..0x4109EF28` 读出 `0x00001602` / `0x02160000` → 显示 **`LF 2.16`**，验证文件：`z8_lf_and_cmdmap.txt`）。
  - 而 `0x410A34FC`（`CMD 0xF3` RX）写入的是另一独立偏移 `[obj_0xb, 0x48..0x4c]`，与机身菜单 `LF` 版本号无关。`V2.16` 统一由 `BORYOZA_LF_BCD_*` 同步导出 `cmd10[0x03..0x06] = 0x02160000` 与 `.readme_ver = 10020b1a`。

### 勘误 (g)：`Sony FE 50-150mm F2 GM`（`0x807B`）`60 Hz` VSync 物理约束与伪 `120 Hz` 规格

- **错误报告**：[BUILD_AND_VERIFICATION_REPORT.md](SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md) 第 66 行及 [BORYOZA_NEW_FW_DESIGN_SPEC.md](HC32F460_SE_NZ_DESIGN_SPEC.md) 第 173 行错误设定 `is_sony_ext` 应切至 `TMRA4->PERAR = 24999`（`120 Hz`）。
- **机器码与总线账本直接反证**：
  - 实测 18 组 `ID05` 线上帧长 `117 B` 加上 `ID06/03/04`、`PA2` 时序及双向响应翻转在 `120 Hz`（`8.333 ms`）下饱和并引发高并发 AF 阻塞；且历史 `AD91` 在 `60 Hz` VD 下错报 `CMD 0x0D = 0x01`（50 Hz）。
  - `V2.16` 在 [lens_session.c](../../se-nz/src/02_core/lens_session.c)（`0x132d2..0x132de`）机器码强制仅对 Sigma 且 `baud >= 1.5 Mbps` 启用 `120 Hz`，实机/SIL 验证 `0x807B` 恒定 `tmra4_perar = 49999`（`60 Hz`）且 `cmd0d = ["00"]`。

### 勘误 (h)：已审计历史报告、测试文档与逆向笔记其余 13 项错误对照表

| 编号 | 错误文档与位置 | 错误断言 | 机器码 / 实测反证与纠正 |
| :--- | :--- | :--- | :--- |
| **h.1** | [BORYOZA_NEW_FW_DESIGN_SPEC.md](HC32F460_SE_NZ_DESIGN_SPEC.md)（L320） | Z 速度字节使用无符号右移 `(hi >> 1)` 解码指数 | `Z8 3.12` `0x40DEE9D8` 与 `V2.16` `0x15a74` 证实为**有符号 7-bit 算术移位** `clamp((int8_t)(hi & 0xFE) / 2, -7, 24)`；无符号解码直接破坏全部 `< 1 pps` 慢速搜索（如 `0x90, 0xFA` = `0.0977 pps`） |
| **h.2** | [BORYOZA_NEW_FW_DESIGN_SPEC.md](HC32F460_SE_NZ_DESIGN_SPEC.md)（L259）与 [Z8_AND_SONY_A1II_A9III_AF_ARCHITECTURE_STUDY.md](Z8_SONY_AF_ARCHITECTURE_STUDY.md)（L110） | 全局硬编码无限远限位 `L_Z = -512` | 无越程余量的镜头（如 `0x807B`）到达物理极限 `0` 后永远满足不了 Z8 `0x40DF0838` 的 `cur_cz <= -512`，引发 `0x40DD0B40` 2.0 s 对焦超时；`V2.15/V2.16` 按镜头分治：`0x807B` 为 `0`，`0xC56E` 保留 `-512` |
| **h.3** | [BORYOZA_NEW_FW_DESIGN_SPEC.md](HC32F460_SE_NZ_DESIGN_SPEC.md)（L293-297） | 允许稳态 `HOT[6]` 保留非零的 bit[5:4]（如 `0x36/0x26`） | `CAPTURE-UPSTREAM-REPORT.md` 与 `Z8 3.12` `410789E0..41078B94` 证实 `HOT[6].bit[5:4]` 非零会导致焦点恢复消费者返回错误码 `1`；置零后返回 `0` |
| **h.4** | [BORYOZA_NEW_FW_DESIGN_SPEC.md](HC32F460_SE_NZ_DESIGN_SPEC.md)（Appendix A）与 `REVIEW.md`（L31-55） | 将 Zeiss Touit `C031/C032/C033` 标为 FX，以及按 `" OS"` 匹配 Tamron `OSD` 镜头开启防抖 | Touit 系列实为 APS-C DX（`0x10`）；Tamron `Di III OSD`（`C133/C134/C135`）仅表示无声马达而无防抖，`CMD 12` 实测为 `00 44` 而非错误推导的 `94 C4` |
| **h.5** | [BUILD_AND_VERIFICATION_REPORT.md](SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md)（L72, 107）、`stress_protocol_matrix.md` 与 [flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) | 陈旧基线描述及硬编码 `V2.07` / `V2.15` 断言 | 当前实装基线已演进至 `V2.17`（`11020b1a`，`LF 2.17`，见下文 §4）；`flash_usb_bor.py:240` 已同步升级至 `"V2.17"` |
| **h.6** | `VR-E-STOP-REPORT.md`（L85 所指正的早期笔记） | 误将 Sigma OS2 指令 `0x53` 解读为“电磁锁中”、`0x5E` 解读为“机械驻车” | Sigma OS2 副 MCU 固件（指令表 `0x16884`）证实 `0x53` 与 `0x5E` 同指向分支 `0x1613B`（`OS mode = 0, Custom = 0`） |
| **h.7** | `OPTICAL_ENDPOINT_BINDING.md`（L93）与 `brand_feature_chains.md` | 早期误将 `C583` 分类为 Single-HLA 架构 | 实机镜头结构与端点契约证实 `C583` 为 **Dual HLA 浮动对焦**（最大放大率 `1:5.4`，最近对焦 `MFD 28 cm`） |
| **h.8** | `unit_profile.md`（L28-31 所纠正的 `dn_optics_consumer.md` 旧表） | 将 `0xC56E` 选择器 `3/4` 误标为 SetA/B（`"R"/"B"`） | 固件地址 `FFF33D58`（含 `FFF18E8B` / `FFF18FF1`）证实真实光学表选择器为：**`2` = 暗角（vignette）、`21` = 畸变（distortion）、`22` = LCA-R、`23` = LCA-B** |
| **h.9** | `VR-INTEGRATION-INPUTS.md`（L90-95） | 早期夹具将 `CMD20 MOSI[2] & 0x06`（`"flag4/flag0"`）误命名为“曝光阶段（exposure phase）” | `Z8 3.12` `41088F80..40EC543C` 机器码证实其为 **Selected / Effective MF 模式位**（合法值 `00/02/04/06`），且 Z8 电子快门期间保持 `B18.bit3 == 0` |
| **h.10** | `reset-source-addendum.md`（L5-6） | 早期仿真器直接归因“RetRAM 奇偶校验错误必然触发 NMI 死循环” | HC32F460 参考手册要求**同时**满足 `CKCR.PYOAD == 0 && INTC_NMIENR.REPENR == 1` 才触发 NMI；而原厂 BL、原厂 APP 与 V207 均将 `NMIENR` 保持为 `0` |
| **h.11** | `brand_source_followup.md`（L22, 49, 79） | 早期假设 `1F08 == mode8`、`C583 3E` 为 `14 B`、`C56F` 近端为 `3200..19600` | 源码复核证伪全部三项：`1F08`（`0x1A72`）`!=` `mode8`（`0x1A56`）；`C583 3E` 净荷实为 `12 B`（`20 B` 线长）；`C56F` 近端限位实为 `5672..8106`（70 mm）/ `6014..8558`（200 mm） |
| **h.12** | `sustained-z-report.md`（L45） | 推论“SPI 外设 `TXIE == 0 && RXIE == 0` 即表示 DMA 没有在工作” | HC32 参考手册第 693 页与 DDL `spi_dma` 例程均通过 AOS 事件矩阵硬件触发 DMA 搬运，完全独立于 CPU `TXIE/RXIE` 与 NVIC 中断使能 |
| **h.13** | `etz21_summary.json` 与 `NOTES.md`（L21） | 早期误判 Boryoza MCU 为 GD32/FM3；`e2nz-v120` `0x1B8A0` 标为“record block”；疑 ETZ21 Pro 随版本动态换密钥 | Boryoza 芯片确证为 **HC32F460JEUA（Cortex-M4F）**；`0x1B8A0` 实为 `55 F0` Z 帧模板池；ETZ21 Pro 加密实为与固件版本无关的静态 per-`(tag, blk)` 变换表 |


---

## 4. `V2.17`（`11020b1a` / `LF 2.17`）合焦确认死区与 `150mm F2` 无限远壁反弹修复补遗

`V2.17` 构建产物：`BORYOZA_V217.PUB`（`157,696 B` / `308` 扇区，SHA-256 `32df26fde8dc3bafca1bcdc2c46009670b8334da8b58da4fc45aab8d7417941d`，`program_end_hex = 0x000367F0`，`.readme_ver = 11 02 0b 1a`，`CMD 0x10[5..6] = 0x17, 0x02` = `LF 2.17`）；[flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) 第 240 行 `--selftest` 版本断言已更新为 `"V2.17"`；验证套件见 `verify_v217_gm50150_confirm_green_and_sweep.py`。

| 编号 | 影响对象 / 根因链路 | 实测现象与 Z8 `3.12` 机器码触发条件 | `V2.17` 修复机制（[af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c)） |
| :--- | :--- | :--- | :--- |
| **KB-16** | `STEP_ACT_ABS` 11-SE 死区挂起 `233 ms` 与误报 `HOT[6]=0x26` 确认失败 | `1D target_se 00 00` 发送后 `af_cmd_pending` 已清零，但 `0x807B` 双 XD 浮动镜组停稳（`hw_moving == 0`）若残差 `> 11 SE`（如 `12..60 SE`）或微确认步位于静摩擦死区内（`af_drive_moved == 0`），`step_active_af_telemetry` 持续挂在 `HOT[6]=0x1D` 达 `14` 个 VSync（`233 ms`）随后发射 `HOT[6]=0x26`（`w9=2`）。Z8 `3.12` `0x40DCC854` 在 `0x1D` 静止计数 `[x19+0x694d] > 4`（即 5 帧）后遇 `w9=2` 跳转 `0x40DCC94C` 置 `*sm = 0x0A`（确认失败、拒转绿并引发二次回跳） | 新增 `closed_loop_moved_tol_se`（`span/192` 钳位 `[24, 64]`）与 `closed_loop_micro_tol_se`（`span/256` 钳位 `[16, 48]`），由 `evaluate_step_settle_ready` 在 `af_cmd_pending == 0 && hw_moving == 0` 后 `2–3` 个稳定 VSync 即平滑收敛至 `HOT[6]=0x06`（`result_code=1`），同时保持真正冻结故障（大误差 `max_slew=0`）继续触发 `0x26` |
| **KB-17** | `AF_SUBMODE_FORCED_STOP` 二次 `0x1D` 抽动与 ETZ21 遗留 `fam2_ext` 反向 `submode=2` 停顿 | `arm_target_drive` 命中 `abs_err <= capture_tolerance_se` 且仍在运动时曾置 `AF_SUBMODE_FORCED_STOP`，先发 `0x1C`、下一帧转 `submode=2`，随后跌入 `format_1d_subrecord` 二次重发 `1D target_se 00 00`；另承自 `ETZ21 0207` `0x0800a5f0` 的 `fam2_ext` 反向 `submode=2` 在发新 `1D` 前先插播 `2–3` 帧 `0x1C` 停顿（`33–50 ms`） | 移除 `AF_SUBMODE_FORCED_STOP`，容差内到位运动直接调用 `issue_controlled_brake` 发射单次 `0x1C`；在 `select_arm_submode` 仅当 `fam2_ext && can_single_shot_direct_abs` 跳过 `submode=2` 预停顿、反向直发 `(0x1D, 0x00)`（同时保留连续相对追焦 `CMD 0xE8` 反向 `0x1C` 预制动） |
| **KB-18** | `150mm F2` 无限远对近物盲搜壁反弹放弃 | 大光圈高倍率完全离焦下无初始 PDAF 相关信号，Z8 `3.12` `0x40DCFA8C` 先探远端壁（`L_Z=0`）后再调用 `0x40DD1FF4 -> 0x40DD1768` 反向扫近：若起点已落在 `lim_far + 1..3 SE`（未置 `cushioned_far`），`arm_target_drive` 早退导致 `0x40DEFE5C()==0`；若双浮动端止光学缓冲停在 `lim_far + 95..240 SE`（原 `span/128 = 93 SE` 带外），超时 `233 ms` 吐出 `HOT[6]=0x26` 触发 Z8 `0x40DD0BB8`（`w8==2 -> *sm=7`）立即放弃；同时原 `MIN_1D_STEP_SE=16` 仅为 `ETZ21` `fcn.0800a024`（`0x20=32`）的一半 | `arm_target_drive` 容差早退分支立即执行 `latch_endstop_on_settle` 并更新 `snap_endstop_cz`；`endstop_cushion_band_se` 扩为 `span/48`（`0x807B` 为 `250 SE`）、`MIN_CUSHION_DRIVE_VSYNCS=2U`、`CUSHION_SETTLE_VSYNCS=2U`；`decide_singleshot_1d` 对高速 Sony XD 端止盲搜置 `SONY_XD_SWEEP_MIN_STEP_SE = 32` |

---

## 5. `V2.18` 与 `V2.19` 架构演进、行为式距离守卫与压缩校准补遗

- **`V2.18` 保留基线**：`BORYOZA_V218.PUB`（`158,720 B` / `310` 扇区，SHA-256 `9a8a515b7f45199d1325eb9175ce8819c37409119b5567b0373950b6c5c84b58`，`.readme_ver = 12 02 0b 1a` = `LF 2.18`），确立无极点 `c0`、光圈独立单次 `(1+beta)^2` 包络与 M4F 热点隔离优化（见 [part4_af_math_and_v218_control_architecture.md](PART4_PDAF_MATH_AND_CONTROL_OPT.md) §1–§4）。
- **`V2.19` 当前发布构建**：`BORYOZA_V219.PUB`（`163,328 B` / `319` 扇区，原始镜像 `163,248 B`，`program_end_hex = 0x00037DF0`，`.readme_ver = 13 02 0b 1a` = `LF 2.19`，`pub_sha256 = a3ca6fe5a3eb98949672bdd429668483b1c2980535d74feb2d505560f65a0ebd`）；[flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) `--selftest` 版本断言已同步升级至 `"V2.19"`。

| 编号 | 影响对象 / 根因链路 | 实测现象与检测准则 | `V2.19` 工程处置与恢复契约 |
| :--- | :--- | :--- | :--- |
| **KB-19** | 零品牌黑名单的行为式对焦距离异常检测与安全回退 | 实拍部分副厂 E 口镜头出现物距编码器 `ID05.dist_code_x` 固定卡在 `255`（`0x00FF`）、无效高值（`0xFF00` / `0x7F00..0xFFFE`）、近端 `cur_pm > 200` 仍报 `0x0000` 或随对焦行程完全冻结的现象；按运行态行为替代品牌/ID 拉黑可保证镜头厂通过后续固件升级修复后立即生效 | [aperture_controls.c](../../se-nz/src/02_core/aperture_controls.c) `aperture_controls_validate_distance` 检测三类物理异常：`bad_reason=1`（`0xFF00` / `0x7F00..0xFFFE` / `cur_pm > 200 && dist_x == 0`）、`bad_reason=2`（`dist_x == 0x00FF` 与物理凸轮位置 `(eff_mfd > 0.32F && cur_pm < 850) || cur_pm < 250` 矛盾）、`bad_reason=3`（采样 `>= 4` 帧且跨过 `>= 220/1024` 归一化行程而 `dist_code_x` 冻结）。触发后置 `dist_report_invalid != 0`、`z_mag_a3_q14 = 0`（保留 3 轴机身防抖 + 镜身 OIS 与 `ID06` 闭环 AF），清零 `CMD 0x10[0x07]`、`CMD 0x12[0x31]`、`CMD 0x20` MF/距离字段与 `plain_exif[0x4E:0x50]`（驱动 Z8 `0x41082B64` 返回 `(-1, -1)`）并写入 `BB_EVT_DIST_FALLBACK (0x000E)`；编码器行为恢复正常时自动退出回退 |
| **V219-OPT-DISC** | 初始化后光学校正断开（Post-Init Optical Calibration Disconnection） | Nikon Z8 在开机初始化获取 `CMD 0x24` 后不再消费运行期动态校正表刷新；断开热重算可消除 `ID05` 光学包抖动并锁固校准快照 | 在 [lens_session.c](../../se-nz/src/02_core/lens_session.c) 与 [z_snapshot_builder.c](../../se-nz/src/02_core/z_snapshot_builder.c) 中，无内置档案变焦首包输出中性 `CMD 0x24` 后立即置 `optics_disconnected = 1, id05.optics_skip = 1`；定焦首帧翻译 `ID05` 1D 曲线后断开并继续维持实时 `delta_av`；同步在 `CMD 0x16` 使能、`CMD 0x15 stage 0`、`CMD 0x24 A+B` 读毕及 repulse 结束锁存并记录 `0x000D` |
| **V219-BB2** | Blackbox v2 `32 B / 256 槽` Flash 滚动记录 | `EFM` 单扇区写满后需原子保留近期事件继续落盘，同时扩展 `arg3` 负载以携带诊断上下文 | [blackbox_recorder.c](../../se-nz/src/03_diag_storage/blackbox_recorder.c) 采用 `32 B` 记录含 `arg3`、32 槽异步环形队列及 `0x0007A000` `256` 槽扇区滚动擦写（[efm_ramfunc.c](../../se-nz/src/00_hal/efm_ramfunc.c) `efm_erase_sector_ramfunc` 允许 `HC32_FLASH_BLACKBOX_BASE`，擦写保留最后 `8` 条有效记录）；新增 `0x000B..0x000F`、`0x00E1..0x00E3` 与 `HARDFAULT` (`BFAR/MMFAR/HFSR`) 故障地址寄存器，并由 [decode_blackbox.py](../../se-nz/tools/decode_blackbox.py) 离线解码 |
| **V219-HUFF** | 两遍式 Canonical Huffman 光学表与 Delta-Nibble 光谱压缩 | 13 组 `CMD 0x24 A/B` 原始档案（`141,622 B`）与 57 条光谱曲线（`4,617 B`）占据大量 `.rodata` Flash 预算 | `gen_lens_db.py` 与 [optics_xlate_fpu.c](../../se-nz/src/02_core/optics_xlate_fpu.c) 将 `Tamron 13` 组压缩至 `50,098 B`（节省 `64.6%`）、`Sigma 150-600` `CMD 0x24 A` 压缩至 `1,638 B`、`57` 条光谱无损压缩至 `2,780 B` + `116 B` 偏移表 |
