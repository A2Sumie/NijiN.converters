# Nikon Z8 (`3.12`) + Native S-Line & Sony `A1 II` / `A9 III` 对焦架构底层逆向研究报告

**研究范围**：基于机器码级逆向拆解，完整对照 **Nikon Z8 v3.12**（`sec2_eg2070_031200.bi`）、三支原厂大虚化旗舰镜头（`MLZ50F12S0110.img`、`F-MLZ400F28TCVRS-V0110M.dmg`、`F-MLZ600F4TCVRS-V0110M.dmg`）及 `MC 105mm f/2.8 VR S` 基线，结合 **Sony A1 II (`ILCE-1M2` V4.00)**（`cpapp-b.bin`）与 **Sony A9 III (`ILCE-9M3`)**（`cpapp-b.bin`）原厂机身固件，定位 **`Sony FE 50-150mm F2 GM` (`0x807B`, `150mm F2.0`)** 在 `BORYOZA V2.14` 实拍大离焦场景下的全部机制根因，并建立 **`BORYOZA V2.15` (`0f020b1a`)** 架构收敛方案（同时完整保留 `Sigma 150-600mm` `0xC56E` 在 `150mm F5.0` 的已验证表现）。

---

## 0. 逆向固件样本清单

| 平台 / 设备 | 固件文件 | 大小 (Bytes) | SHA-256（前 16 hex / 全长核验） | 地址映射基址 |
|---|---|---|---|---|
| **Nikon Z8 v3.12** | `sec2_eg2070_031200.bi` | `81,807,711` | `0a0a0afda5d11049bbbbab698c8db88223fda6ecf2d1231c68aa8f8f93b0a09a` | `ROM VA = off + 0x40160000`<br>`initRAM VA = off + 0x9FDCF5000` |
| **NIKKOR Z 50mm f/1.2 S v1.10** | `MLZ50F12S0110.img` | `872,200`（容器）<br>解包 ROM `851,968` | 容器 `e8db7ae9a3d8c220...`<br>ROM `b0661073e4df47f6...` | RXv2 `[0xFFF00000, 0xFFFD0000)` |
| **NIKKOR Z 400mm f/2.8 TC VR S v1.10** | `F-MLZ400F28TCVRS-V0110M.dmg` | `789,669`（容器）<br>SSVCM `425,984` / Main `1,835,008` | 容器 `a37ab6f687553848...`<br>bank0 `904564c60ec6f97f...`<br>bank1 `0e268bc01d0ce764...` | RXv2 `bank[0]` `[0xFFF80000, 0xFFFE8000)`<br>`bank[1]` `[0xFFE00000, 0xFFFC0000)` |
| **NIKKOR Z 600mm f/4 TC VR S v1.10** | `F-MLZ600F4TCVRS-V0110M.dmg` | `789,565`（容器）<br>SSVCM `425,984` / Main `1,835,008` | 容器 `d5c2c0acf8d606f9...`<br>bank0 `93d0c6f34318496f...`<br>bank1 `0a87de525dd3ecd9...` | RXv2 `bank[0]` `[0xFFF80000, 0xFFFE8000)`<br>`bank[1]` `[0xFFE00000, 0xFFFC0000)` |
| **Sony A1 II (`ILCE-1M2` V4.00)** | `cpapp-b.bin` | `19,190,024` | `96b41427958c1b412b3e15c5bd9b51e34657ae1e1246344438b8368031ca4859` | ARM32 Thumb-2 `0x00108000` |
| **Sony A9 III (`ILCE-9M3`)** | `cpapp-b.bin` | `20,872,312` | `5da0e9124aa927f2b2fc7b47fb362522cd617fd647a4ccd45e34a5784ac675fe` | ARM32 Thumb-2 `0x00108000` |

---

## 第一部分：Nikon Z8 `3.12` + 原生 S-Line 完整对焦架构

### 1.1 顶层 AF 状态机与 `CMD 0xC0 / E2 / E4 / E6` 分发架构

Z8 主 AF 控制循环位于 `0x40DC28C0..0x40DC3710`，核心上下文 `af_ctx` 包含 89 区相关器结果缓冲 `*(af_ctx + 0x00)`、镜头参数结构 `lens_af = *(af_ctx + 0x20)`（传给 `0x40DF0A28` 的 `arg2` 为 `lens_af + 4`）、机身模式 `mode_ctx = *(af_ctx + 0x28)` 与状态块 `sm = *(af_ctx + 0x30)`（`*sm = 0x00/0x01` 待机触发，`0x02..0x04` 闭环跟踪，`0x10` 合焦锁定，`0x11` 大散焦连续扫描搜索，`0x0B/0x0C/0xFE` 终止态）。

在 `0x40DCA610` 中，原厂与转接环均上报 `prop 0x157 = 1`（`0x40DCBBE8() == 1`），因此在总线分发器 `0x40DCDB2C` 中，Z8 严格按触发场景选取以下指令与速度档位：

| 对焦阶段与 Z8 触发 VA | `0x40DCDB2C` `case` | 线上 SPI 指令 | 目标与速度编码特征（直接关系 `BORYOZA V2.15`） |
|---|---|---|---|
| **全盲大散焦连续扫描搜索**<br>（`0x40DCFCA0 -> 0x40DD0454 -> 0x40DCFA8C`，`*sm = 0x11`） | `case 3`（Z 卡口走 `0x0B -> 0x40DED9F4`） | **`CMD 0xC0`** (`18B`) | **`mosi[1..4] = 0`**；**`mosi[5..6]` 为按 $F\#$ 限速的有限搜索速度 `vpps`**；`mosi[7..8] = 0xFF 0x3F`（照片）或有限速（视频）；`mosi[11] = 0x2D`；`mosi[12] = cookie`；**`mosi[13..16] = wall_cz`**（`C_Z_near`、`C_Z_far` 或 `±0x7FFFFFFF`）。 |
| **视频 / Live View 平滑伺服逼近**<br>（`0x40DCE078 -> 0x40DCE154`，`mode+0x70 == 1`） | `case 0x0F` | **`CMD 0xC0`** (`18B`) | `mosi[1..4] = 0`；`mosi[5..6]` 与 `mosi[7..8]` 均为受限平滑速度 `vpps`；`mosi[13..16] = target_cz`。 |
| **静态照片单次全速跳转**<br>（`0x40DCE078`, `0x40DCE3C4`） | `case 5 / 6` | **`CMD 0xE6`** (`10B`) | `mosi[1..4] = target_cz`；默认 `+0x1A34 == 0` 时 `mosi[5..6] = 0xFF 0x3F` 全速哨兵；`mosi[8] = cookie`。 |
| **扫描途中捕获焦点重定向**<br>（`0x40DD40A4 -> 0x40DD1FF4 -> 0x40DD1768`） | `case 0x0D` / `10` / `0x0E` | **`CMD 0xE0` 后发 `CMD 0xE2`** (`10B`) | 先发 `E0 [cookie]` 停稳（等待 5 采样 `(HOT[6] & 0x08) == 0`），随即在 `E2` 写入 `mosi[1..4] = target_cz` 与 `0xFF 0x3F`（或有限速）直接落位；`mosi[8] = cookie`。 |
| **`AF-C` 连续追焦与失锁巡航**<br>（`0x40DCE78C`, `0x40DCE9F0`） | `case 9 / 12` | **`CMD 0xE4`** (`16B`) | `mosi[1..4] = cur_cz`（`0x40DB3F90`）或前瞻位移；`mosi[5..6]` 为有限有符号速度 `spd1`；`mosi[10] = cookie`；`mosi[11..14] = wall_cz`。 |

---

### 1.2 大散焦 $F\#$ 搜索速度律、`CMD 20[0x85..0x8C]` 与“直接开始卡” `2.0s` 死锁机制

1. **四级搜索速度公式（`0x40DD27B4 -> 0x40DD29E4 -> 0x40DD2EC8 -> 0x40DD34D8 -> 0x40DD36A0`）**：
   - 第一级按开放光圈 $F_{\text{q8}} = \min(256 \cdot F_{\text{open}}, \text{0xB00})$ 与传感器帧率 $\text{fps}$ 确定基准像面散焦推进速率：
     $$v_{\text{def,base}} = \left\lfloor \frac{F_{\text{q8}} \cdot G_{\text{rate}} \cdot \text{fps}}{1280} \right\rfloor$$
     因为相位相关捕获窗口厚度正比于景深 $\propto F\#$，因此在 `150mm F2.0`（$F_{\text{q8}}=512$）与 `50mm F1.2`（$F_{\text{q8}}=307$）下，Z8 **刻意大幅降低每帧脉冲位移**，使焦点穿过狭窄 PDAF 捕获带时至少在多个连续传感器帧内保持可相关信号，从而在 `0x40DD40A4` 完成中途锁焦。
   - 第三级通过 **`CMD 20[0x85..0x88]` (`prop 0x102`, `u32 LE` `max_vpps_stills`)** 与 **`CMD 20[0x89..0x8C]` (`prop 0x103`, `u32 LE` `max_vpps_movie`)** 执行无符号除法（`0x40DD2914..0x40DD2968` `udiv`）：
     $$v_{\text{def,lens\_max}} = \left\lfloor \frac{\text{max\_vpps} \ll 8}{\lfloor K_Z / 256 \rfloor} \right\rfloor$$
     与全行程 $7$ 帧时间下限 $|C_{Z,\text{near}} - C_{Z,\text{far}}| \cdot \text{fps} / 7$ 取最小值封顶。`V2.14` 将 `CMD 20[0x89..0x8C]` 硬编码为 `1000` (`0x03E8`) 并将 `[0x8D]` (`prop 0x0FF`) 置 `1`，直接把 LV / 视频态全行程搜索扼杀至几十秒。

2. **Z 卡口双字节速度浮点（`0x40DEE9D8`）的有符号 7-bit 指数编码**：
   - `0x00, 0x00` 表示 `0.0 pps`；`0xFF, 0x3F` 表示全速无限制。
   - 通用非零速度 pair `[spd_lo, spd_hi]` 的高字节 `spd_hi` 定义为 `((exp7 << 1) | sign_bit)`，其中 **`exp7 = ((int8_t)spd_hi) >> 1` 是二进制补码有符号 7 位指数（`[-7, +24]`）**：
     $$\text{vpps} = (256 + \text{spd\_lo}) \cdot 2^{\text{exp7} - 9}$$
   - `F1.2` / `F2.0` 超慢扫速（例如 `0x19` = `0.0977 pps`，线上字节 `[0x90, 0xFA]`）天然置位 `spd_hi` 的 `0x80` 位（`(int8_t)0xFA >> 1 = -3`）。若把 `bit 7` 误判为固定常量，超低速会被放大上百倍导致错过窗口。

3. **Z8 `3.12` 原生 Z 卡口端止挡折返判据与 `0x40DD0B40` 2.0s 卡死机制（`0x40DF0838` & `0x40DF08EC`）**：
   - 当 `prop 0x157 == 1`（Z 卡口原生/转接环）时，远端止挡检测 `0x40DF0838` 与近端止挡检测 `0x40DF08EC` **直接跳过 `HOT[7]`，仅以 `C_Z` 数值等号对比 `CMD 20` 声明的 `[L_Z, H_Z]`**：
     - `0x40DF0838(lens_af+4, cur_cz * 10)`：返回 **`(C_Z_far * 10 >= cur_cz * 10)`**（即 `cur_cz <= L_Z`）；
     - `0x40DF08EC(lens_af+4, cur_cz * 10)`：返回 **`(C_Z_near * 10 <= cur_cz * 10)`**（即 `cur_cz >= H_Z`）。
   - 在 `*sm = 0x11` 搜索态下，一旦 `0x40DD0B40` 检测到镜头连续 **5 帧** `HOT[2..5]` `C_Z` 不再前进，但 `0x40DF0838` / `0x40DF08EC` 仍返回 `0`（例如 `L_Z = -512` 而停在 `cur_cz = 0`，或近端因双浮动/XD 死区缓冲停在 `40,940 < mech_hz = 40,960`），Z8 拒绝触发端点折返，锁存时间戳原地等待 **`2,000,000 µs`（2.0 秒）**后才强杀退出。

---

### 1.3 原生旗舰镜头 (`50/1.2 S`、`400/2.8`、`600/4`、`MC 105`) 实测光学/光瞳/伺服对照与 Z8 消费数学

1. **Z8 `3.12` 光瞳→散焦→有理连分式 `K_Z` 全链路与 F 值无关性**：
   - Z8 首先通过三平面猫眼射线裁剪模型（`0x40DF29B0 -> 0x40DF2BB0`）计算离轴有效光圈 $F_{\text{eff,q8}}$ 与质心偏移 $C_{\text{shift}}$：
     - $R_k = |256 p_m / (2 F_k)|$，$S_k = \text{trunc}((256(p_k - p_m)/p_k)\cdot |x|/16)$，$L = \max_k(S_k - R_k)$，$U = \min_k(S_k + R_k)$。
     - 再由 `0x40DF9B30 -> 0x40DF9538` 利用 $F_{\text{eff,q8}}$ 将原始像素视差换算成物理像面散焦量 `defocus_nm`，经 `0x40E15F68` 转为微米 `d_um` 后送入二阶有理连分式求解器 **`0x40DB76B0`**（结合 `0x40DB7DDC` 解码的无符号浮点 `K_Z = ((256 + b0) << b1) >> 1` 与 `0x40DB7F88` 解码的有符号斜率 `c69`）。
   - **原生实测铁证**：
     - `50/1.2 S`（$F1.2$）与 `MC 105/2.8`（$F2.8$）在 $\infty$ 均满足 $K_Z \cdot f^2 \approx 2^{24}$；
     - `400/2.8`（$431.0 \to 216.0\text{ pulses/mm}$ @ `1.4x TC`）与 `600/4`（$383.0 \to 191.5\text{ pulses/mm}$ @ `1.4x TC`）切入增倍镜时凸轮机械量不变，$K_Z$ 严格按 $1.414^2 = 2.00\times$ 缩放。这从机器码与实测数据两面共同确证：**Z 卡口 `K_Z` (`CMD 20[0x6D..0x6E]`) 与光圈 $F\#$ 完全解耦**。
2. **原生圆形通光孔径猫眼窗口特征**：
   - `50/1.2 S`、`400/2.8`、`600/4` 全部对 **`[0x21..0x24]` (`h3, h4`) 填 `00 00 00 00`**，且在大光圈/大入瞳端同步启用 `Plane 2`（前镜筒）与 `Plane 3`（卡口喉部）；只有微距方形遮光框 `MC 105/2.8` 才填 `h3 = -65, h4 = -277`。
3. **原生 SSVCM (`400/2.8` & `600/4` `bank[0]`) 与双浮动 (`50/1.2 S`) 伺服减速律**：
   - `400/2.8` / `600/4` 的 SSVCM 独立子 MCU 运行 `1 kHz` **五次多项式最小加加速度（$C^2$ Minimum-Jerk）6 段规划器**（`0xFFF8D742 -> 0xFFF8F2B6`，终点强制 $v(T)=0, a(T)=0$），并按当前 $(x, v, a)$ 实现空中反向连续平滑过渡与误差分段增益调度 PID。

---

## 第二部分：Sony A1 II (`ILCE-1M2`) & A9 III (`ILCE-9M3`) E 卡口对焦架构

### 2.1 `CMD 0x1D` vs `CMD 0x43 / 0x6B / 0x3C / 0x49` 与 `120Hz` 协议验证

1. **`CMD 0x1D` (`A1 II 0x432A80` / `A9 III 0x435E0C`, `setMoveFcCmd 0x42D778`) 与 `CMD 0x43` (`0x432B04`)**：
   - `CMD 0x1D`（`5B`：`1D b1 b2 b3 ctrl_b4`）不含任何硬件速度字段：`b4 = 0x00` 为终点驻停绝对定位，`b4 = 0x80` 为 `AF-C` 连续绝对目标跟随，`b4 = 0x2C/0x2E` 为 `s16`（及微抖振幅 `b3`）步进，**`b4 = 0x3C` 为 `family=2` 专属 `s24`（`b3 = 0x00/0xFF`）有符号相对步进**。
   - 对 `CMD 0x43`（`12B`：`seq, abs_se, rel_shift, wob, ctrl_flags, cnt, fc_start_time`）反汇编证实：**`0x43` 的运动学载荷与 `0x1D` 完全同构，同样不含速度/加减速曲线字段**（直到 `A9 III` 新增 28 字节 `CMD 0x6B` `@ 0x435FA4` 才加入 5 点轨迹数组）。在 `A1 II` `0xAF6892` 中，当 `cap(0x43) == 0` 时，Sony 机身原生即通过 `ID04` 内的 `0x1D`（`b4 = 0x3C` + `0x80/0x00`）驱动 `family=2` XD 线性马达，借助每 VSync 受控步进规划外部速度/减速斜坡。
2. **`1500 kbps` (`0C 02`) 与 `CMD 0x0D` 基频核验**：
   - `0xB31356` 按 `[id01+0x22..0x23] & 0x0200` 选 `0C 02`（`1500 kbps`）。
   - 结合 `sony-base-rate-results.json` Unicorn 原码实证：`CMD 0x0D` 参数 `00 = 60Hz`, `01 = 50Hz`, `02 = 48Hz`（即基准帧频选择），动态高帧率通过 `CMD 0x5F` 协商；同时 `ID03` `wire[13]`（`0xB20AAE`）恒保持 `0x1C` 不随 VSync 改变。因此 BORYOZA 保持 `0D 00` (`60Hz`) + `CMD 32/5A` + `ID04+1D` 是确定性最高、半双工时序最安全的路线。

### 2.2 `ID06 defocus_k_se` 精确公式与光圈/放大率校准证明

通过追踪 `A1 II` `0xAA2A32`（`A9 III` `0xAA4084`）机器码：
- **`ID06 tag 0x44`**（`0x137A68`）：扩展标签直接上报纯几何灵敏度 $K_{\text{ext}} = 2 \cdot d\text{SE}/dz_{\text{mm}}$（无关 F 值），当前 BORYOZA 采用标准 16B `ID06` 掩码，故镜头运行于下一通路。
- **标准 `ID06[13..14]` (`raw_k`)**（`0xAA2A32`）：
  $$\Delta\text{SE} = \text{round}\!\left(\frac{32 \cdot \Delta z_{\mu\text{m}} \cdot \text{raw\_k}}{\text{round}(384 \cdot F_{\text{cur}})}\right) = \text{round}\!\left(\frac{\Delta z_{\mu\text{m}} \cdot \text{raw\_k}}{12 \cdot F_{\text{cur}}}\right)$$
  **双重定谳结论**：
  1. Sony 标准 `ID06.defocus_k_se`（`raw_k`）在镜头 MCU 内部**已经预乘了 $0.012\,\text{mm} \cdot F_{\text{cur}}$ 且反映当前焦点/变焦下的实时像面灵敏度 $\frac{d\text{SE}}{dz_{\text{img}}}$**；
  2. 因此 [af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c) 的 `calibrated_emount_k` **必须保留对 `fnum_q8` 的除法**以消除 $F_{\text{cur}}$ 因子（从而对齐 Z8 与 F 值无关的 `K_Z`），但在 `af_servo_refresh_kz`（[af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c)）中，Sony 原厂镜头无需额外再除以 $(1+\beta)^2$（否则在近摄端会人为压低 `K_Z`）。

### 2.3 Sony 8 重 `SCANLESS` 门控与近目标二分阻尼减速律

`A1 II` `0x135270..0x135830` 在大散焦/高多径噪声（`ympc`）/低对比度（`cpeak`）时拒绝直接无扫描跳焦，转入受控速度推进，并在 `0xAA8F8E..0xAA8FD8` 接近目标时执行平滑阻尼减速，进入 $\pm 1.5 F\delta$ 捕获带后切入绝对目标带滞回锁定。

---

## 第三部分：`0x807B` (`150mm F2.0`) 失稳根因与 `BORYOZA V2.15` (`0f020b1a`) 工程方案

综合双生态机器码比对，实测中 `Sigma 150-600mm @ F5.0` 可靠而 `Sony 50-150mm F2 GM @ 150mm F2.0` 大离焦不稳的核心原因如下：

1. **`can_single_shot_direct_abs` (`|| fam2_ext`) 绕过 Z8 `case 3` `CMD 0xC0` 大离焦限速扫描**：`V2.14` 中 `fam2_ext` 对 Sigma 为 `false`、对 `0x807B` 为 `true`，导致 Z8 在大离焦下刻意按 `F2.0` 降低搜索速度发出的 `CMD 0xC0` 在 `0x807B` 上被全部转换成不限速的 `1D wall_se 00 00` 全速瞬移，马达以全速两三帧掠过极窄的 `F2.0` PDAF 窗口而彻底致盲 `0x40DD40A4`。`V2.15` 仅当 `unlimited_speed != 0` 或 `cmd_vel_se_pps >= vmax_tps` 时走直接绝对定位，严格保留 Z8 限速指令通过 `0x3C` 平滑推进。
2. **原生 Z 卡口止挡协调与 `0x40DD0B40` 2.0s 卡死消除**：当镜头物理抵达/接近无穷远（无越过无穷远余量）或带死区缓冲近端止挡停下时，将真实可达远端置 `mech_lz = 0`（而非无端 `-512`）并在静止贴壁时将上报 `cur_cz` 对齐声明边界 `[mech_lz, mech_hz]`，使 Z8 `0x40DF0838` / `0x40DF08EC` 立即判定触壁折返，消除 `2.0s` 停滞卡死。
3. **`CMD 0xC0` 哨兵墙与有符号 7-bit 速度指数解码**：在 `CMD 0xC0` 中为 `mosi[13..16]` 接入 `resolve_sentinel_target` 及 `mosi[1..4]` 相对回退，按 `exp7 = ((int8_t)(spd_hi & 0xFE)) / 2` 解码全部正负指数速度 `vpps`。
4. **1 帧 `af_predicted_se` 前瞻 + 末端平滑减速斜坡 & `K_Z` 去重衰减**：在 `decide_continuous_1d` 与 `decide_singleshot_1d` 对 `fam2_ext` 加入 `eval_fam2_approach` 靠近收敛衰减，使高速双 XD 线性马达无过冲落入 `target_se`；同步移除 `LENS_BRAND_SONY` 下的重复 `(1+\beta)^2` 除法。
5. **`CMD 20[0x85..0x8D]` 真实 `max_vpps` 与 `150mm F2.0` 三平面猫眼窗口**：将 `[0x85..0x88]` 置 `65,536 pps`、`[0x89..0x8C]` 置 `16,384 pps`（消除 `1,000 pps` 视频/LV 搜索窒息）、`[0x8D]` 动态随限位开关清 `0`；对高通光 Sony 远摄端（`>= 85mm`, `<= F2.4`）启用 `Plane 2 + Plane 3` 圆形三平面猫眼窗与 `h3 = h4 = 0`，提升轴外与大模糊圈条件下的子光瞳基线精度。
