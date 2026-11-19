# Part 4：Z8 PDAF 机器码数学拆解、`ETZ21` vs `V2.17` 闭环瓶颈诊断与 `V2.18` 控制/MCU 优化架构

**文档范围**：本卷基于 Nikon Z8 `v3.12`（`sec2_eg2070_031200.bi`，`baddr = 0x40160000`）AArch64 原生求解器机器码、`Megadap ETZ21 Pro Plus 0207` ARM Cortex-M4 固件反汇编、`BORYOZA V2.17`（[af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c) / `BORYOZA_V200.elf`）以及新构建的真实 Q8-mm 闭环基准测试 `benchmark_defocus_closed_loop_etz21.py`，完整推导 `V2.18`（`LF 2.18` / `12020b1a`）的 AF 控制数学模型与保守 HC32F460 性能优化方案。

---

## 1. Z8 `3.12` 有理式 PDAF 内核与 `0x807B` 实拍失效根因的精确数学拆解

### 1.1 Z8 `0x40E15E58 -> 0x40DB76B0` 的精确输入尺度与方向符号

直接对照 Z8 `3.12` 指令序列 `0x40E15F50..0x40E15FB8` 与 `0x40DB76B0` / `0x40DB7F88`：

1. **焦面离焦输入单位（`Q8 mm`）与目标符号约定**：
   - `0x40DB76B0(coeff[3], d_q8, &out_q16)` 在 `c0 != 0, c1 == 0` 时严格执行：
     $$D_1 = 256 + q(c_0, d_{\mathrm{q8}}) = 256 + \mathrm{sign}(c_0 d_{\mathrm{q8}})\left\lfloor\frac{|c_0 \cdot d_{\mathrm{q8}}|}{256}\right\rfloor = 256 + c_0 \cdot d_{\mathrm{mm}}$$
     $$\Delta C_{Z,\mathrm{q16}} = \mathrm{trunc}\!\left(\frac{K_Z \cdot d_{\mathrm{q8}}}{D_1}\right), \qquad \Delta C_Z = \Delta C_{Z,\mathrm{q16}} \gg 16$$
   - `0x40E15FB4` 计算 `target_cz = cur_cz - ((pulses_q8_main + pulses_q8_aberr) >> 8)`。因此当镜头位于无穷远（`cur_cz = 0`）并合焦于近处目标（`target_cz > 0`）时，`d_q8 < 0`（后焦）；当从近处返回无穷远（`target_cz = 0`）时，`d_q8 > 0`。
   - `0x40DB7864` 对 64 位有符号除法执行 **32-bit 饱和截断 `[-32768, +32767]`（对应 Q16 整数脉冲突界）**。

2. **`ETZ21 0207` `c0 = -5`（`0xAE 0xFB`）极点悖论**：
   - `ETZ21 0207` 在 `CMD 20[0x63..0x64]` 与 `[0x69..0x6A]` 固定写入 `ae fb`（经 `0x40DB7F88` 解码得 $c_0 = -5$）。
   - 当离焦为负（无穷远 -> 近处，$d_{\mathrm{mm}} < 0$）时，$D_1 = 256 - 5\,d_{\mathrm{mm}} > 256$，压低初始跳跃步长，抑制近摄非线性过冲。
   - 然而当离焦为正（近处 -> 无穷远，$d_{\mathrm{mm}} > 0$）时，$D_1 = 256 - 5\,d_{\mathrm{mm}}$ 在 **$d_{\mathrm{mm}} = +51.2\;\mathrm{mm}$** 处存在奇异极点（$D_1 = 0$）；当总散焦行程接近或超过 `+51.2 mm` 时，$D_1 < 0$ 导致 `0x40DB76B0` 输出符号反转，使镜头反向撞向近摄端止（已在 `benchmark_defocus_closed_loop_etz21.py` `S2` 实测重现）。

### 1.2 `V2.17` 在 `Sony FE 50-150mm F2 GM`（`0x807B`）上的四重数学缺陷

| 缺陷编号 | 触发位置（`V2.17` 源码） | 精确数学机制 | 实拍现象与基线实证 |
|---|---|---|---|
| **D1：近摄双倍 `(1+beta)^2` 过阻尼** | `af_servo_bridge.c:819-822`（`af_servo_refresh_kz`） | Sony 原生内对焦镜头在 `ID06.defocus_k_se` 中已随凸轮斜率按 $(1+\beta)^{-2}$ 物理衰减；`V2.17` 无条件再除 $(1+\beta)^2$，等效按 $(1+\beta)^{-4}$ 衰减（近摄时仅剩真实灵敏度 `~41%..68%`） | `S1`（`150mm F2` 0.75m）与 `S5`（`50mm F2` 0.40m）出现 **`5` 次连续停走二次微步**（`4637 -> 1291 -> 389 -> 118 -> 35 -> 12 um`），拖长确认延迟，超 `15` 帧窗口仍不能转绿（实拍“抽一下能对上，长确认不转绿”） |
| **D2：长焦端盲搜雅可比耦合与近摄双除叠加** | `lens_db_generated.h` + `af_servo_bridge.c` | `0x807B` 登记 `MFD = 0.40 m`（`mech_hz = 40,960`）以保持 `3.413 C_Z/SE` 柔顺 CDAF 盲搜雅可比；在 `150mm` 端叠加 `D1` 过陡的 `c0 = -5` 与近摄双除 `(1+beta)^2` 后，单跳落入 Z8 `0x40DB7864` Q16 `32,767` 上限并把确认段拖成多轮慢追赶 | `0x807B` 保持 `mech_hz = 40,960`（维持 `50->150mm` 变焦收缩与 `8192 C_Z/s` 盲搜平稳步进），改由无极点 `c0 = -1` 与光圈独立单次 `(1+beta)^2` 包络收拢剩余小残差并 1 帧转绿 |
| **D3：`is_dual_floating` `40 SE` 吞指令死锁** | `af_servo_bridge.c:545`（`span_tolerance_se`） | `is_dual_floating != 0` 直接令 `capture_tolerance_se() = 40 SE`。`150mm F2.0` 一倍焦深仅 `±4.5 SE`，当 Z8 在 `arm_target_drive` 下发 `4..39 SE` 细确认步进时被吞没、`0x1D` 零发送却回报 `HOT[6]=0x06` | `S7`（`0xC56F` `18 SE` / `124.2 um` 残差）连续 **`12` 次确认重发全被吞没**，镜组纹丝不动，最终触发 Z8 `*sm = 7` 放弃合焦 |
| **D4：`E4` `delta_cz` 被旁路与陈旧 `cur_cz` 锚点** | `af_servo_bridge.c:1368-1377, 1531-1542` | `handle_z_cmd_e4` 先判断 `mosi[9] != 0` 而覆盖非零 `delta_cz`；同时 `relative_focus_target` 直接累加物理滞后样本 `servo->cur_cz` 而非曝光同步热线 `last_hot_cz` | `S3`（`E4 mosi[9]=0x29, delta_cz != 0`）忽略相对步长直接狂冲端止，`peak_overshoot_um = 29,357.8 um`、`final_err_se = 5240 SE` |

### 1.3 `V2.17` vs `ETZ21 0207` 基线闭环实测（`build/v217_vs_etz21_baseline.json`）

在一致非线性光学凸轮 + 二阶双 XD 惯性模型（`vmax = 2400 SE/VSync`, `adec = 1400 SE/VSync^2`, 绿框门槛 `|d_um| <= F * 16.0 um` 连续 5 帧且在快拍帧预算内）下，基线测试结果如下：

- **`ETZ21 0207`**：`1/7` 绿框锁定（`S7` `6` 帧完成 `err = 0`；`S1/S5` 受 `regime 3` 单帧 `512 SE` 上限制约 `15` 帧只走到 `8573 SE`；`S2` 因 `c0 = -5` 越过 `+51.2 mm` 极点反转撞向 `13000 SE`）。
- **`BORYOZA V2.17`**：`2/7` 绿框锁定（`S2` `15` 帧、`S4` `10` 帧；`S1/S5` `5` 次确认蠕变超时、`S3` `E4` 过冲 `29.36 mm`、`S6` 过冲 `6.65 mm` 残差 `44.1 um` 超焦深、`S7` `12` 次确认被 `40 SE` 容差吞没）。
- **`V2.17` HC32F460 基线指令数**：`10` 个稳态 VSync + `CMD 0x20` 轮询共执行 **`5,403,703` 条指令**（**`540,370.3` insns / VSync**）。

---

## 2. `V2.18` 控制数学与算法架构升级

### 2.1 无极点焦距自适应牛顿有理系数 `c0` + 单次 `(1+beta)^2` 参考包络

1. **保留 `0x807B` 柔顺 CDAF 雅可比与单次光圈独立倍率包络**：
   - `0x807B` 保持 `mfd_meters = 0.40`（`mech_hz = 40,960`、`3.413 C_Z/SE`），确保长焦 `8192 C_Z/s` 盲搜每帧收敛在约 `40 SE/VSync` 且 `50 -> 150 mm` 变焦全量程平滑收缩；大离焦主跳由 `c0 = -1` 结合单次包络压进确认窗，末端 `0x40DB7864` 微小饱和缺口在 1 帧确认期内直接锁绿。
   - 为严格解耦光圈缩放与近摄倍率衰减，无穷远参考 `ke_inf_ref` 采样**未经光圈校正**的原始 `id06.defocus_k_se`（随焦点/变焦重置；仅当 `grp_present[4]` 就绪且 $\beta < 0.03$ 时首次锁存 `raw_k * (1+\beta)^2`，后续在无穷远仅允许向上包络或静止一致刷新以防乱序 `ID06` 污染；若从近摄冷启动则需 $(1+\beta)^2$ 杠杆比 $\ge 1.25$ 且缩放模型残差小于常值模型一半才解锁）。再对光圈灵敏度输出 `calibrated_emount_k(ctx, raw_k)` 乘以无量纲单次缩放因子 `compute_single_beta_scale(ctx)`：
   $$K_{E,\mathrm{cap,raw}} = \frac{K_{\mathrm{raw,inf\_ref}}}{(1+\beta)^2}, \qquad \mathrm{beta\_scale} = \min\!\left(1.0,\,\frac{K_{E,\mathrm{cap,raw}}}{\max(1,\,K_{\mathrm{raw}})}\right), \qquad K_{E,\mathrm{eff}} = \mathrm{calibrated\_emount\_k}(K_{\mathrm{raw}}) \cdot \mathrm{beta\_scale}$$
   - `F2.0 -> F2.8 -> F8.0` 缩光圈严格保留 $1/F$ 比例缩放，同时近摄对原厂不自降 `k_se` 镜头自动施加单次 $(1+\beta)^2$ 包络、对已自降 `k_se` 镜头零双除。
   - 对真实 Sony `ID06.defocus_k_se` 已按 $(1+\beta)^{-2}$ 衰减的镜头：$K_{E,\mathrm{raw}} \le K_{E,\mathrm{cap}}$ 恒成立，`V2.18` **不再重复除**，一次跳步直达焦深内。
   - 对恒定 `defocus_k_se` 测试模型（如 `verify_v216` Gate 2）：$K_{E,\mathrm{raw}} > K_{E,\mathrm{cap}}$，自动精确钳位到 $(1+\beta)^{-2}$ 倍率阻尼。

2. **无极点安全牛顿有理系数 `kz_c0_b0 / kz_c0_b1`**：
   先将 `K_raw,inf` 换算为**光圈无关**的规范 `F2.0` 基准灵敏度 `ke_inf_f2_base`（`= k_inf_raw * 9 / 35`），再乘以近摄二次积分膨胀系数 $m_{\mathrm{int}}(\beta) = \max(1.20,\,1 + \beta + \beta^2/3)$，得全行程像方离焦上界 $d_{\mathrm{mm,max}} = (\text{abs\_span} / K_{E,\mathrm{inf,F2}}) \cdot m_{\mathrm{int}}(\beta)$（支持 `8 mm` 及以上超广角真实焦距）：
   $$c_{0,\mathrm{f}} = \mathrm{round}\!\left(\frac{160.0}{f_{\mathrm{mm}}(1+\beta)}\right), \qquad c_{0,\mathrm{safe}} = \left\lfloor\frac{150.0}{d_{\mathrm{mm,max}}}\right\rfloor, \qquad |c_0| = \mathrm{clamp}\!\left(\min(c_{0,\mathrm{f}}, c_{0,\mathrm{safe}}),\,0,\,5\right)$$
   - 极点安全性保证：$D_1 = 256 - |c_0|\,d_{\mathrm{mm,max}} \ge 256 - 150 = +106 > 0$（全物理行程与 `F2.0..F22` 全光圈档位保持正分母零极点）。
   - 对于模拟 `k_se = 120` 的超大虚行程单元测试（$d_{\mathrm{mm,max}} > 300\;\mathrm{mm}$），$c_{0,\mathrm{safe}} = 0$ 自动输出 `0x0100` 纯线性；对真实 `50-150mm` 在 `150mm` 端（`k_se = 960`）恒定输出 `c0 = -1`（`0x80 0xF9`，`D1(+60mm) = 196 > 0`），在 `50mm` 广角（`k_se = 2560`，$d_{\mathrm{mm,max}} \approx 21.9\;\mathrm{mm}$）输出 `c0 = -3`（`0xC0 0xFA`），在 `Inf -> Near` 大跳时适度抑制峰值过冲并在近->远保持单调安全分母。
   - 按 Z8 `0x40DB7F88` 编码：`-1 -> (0x80, 0xF9)`、`-2 -> (0x80, 0xFA)`、`-3 -> (0xC0, 0xFA)`、`-4 -> (0x80, 0xFB)`、`-5 -> (0xA0, 0xFB)`、`0 -> (0x00, 0x01)`。

### 2.2 细粒度命令捕获与稳态停稳解耦 + 热线暴露锚点对齐

- **发送前捕获阈值**：从 `span_tolerance_se` 中移除 `is_dual_floating` 的 `40 SE` 强行放大，恢复全平台 `capture_tolerance_se = clamp(abs_span / 3072, 2, 6)`（`0x807B` 为 `3 SE`），使 Z8 任何 `>= 4 SE` 细确认微步都能直发驱动，消除 `S7` 吞指令假死；同时保留 `evaluate_step_settle_ready` 中指令发完（`af_cmd_pending == 0`）后的 `closed_loop_moved_tol_se`（`24..64 SE`）与 `closed_loop_micro_tol_se`（`16..48 SE`）用于吸收双 XD 机械静摩擦死区，确保绝不误发 `0x26`。
- **相对移动热线锚点**：`af_servo_build_hotline_frame` 记录最新推算热线坐标 `last_hot_cz`（静止时与 `cur_cz` 同步）；`relative_focus_target` 以 `(servo->hotline_seen != 0U) ? servo->last_hot_cz : servo->cur_cz` 为基准叠加 `delta_cz`，与 Z8 `0x40DEE3EC` 积分窗及 `ETZ21 0x0800a5f0` 严格对齐。
- **`CMD 0xE4` 优先级修正**：先判定 `delta_cz != 0` 并叠加双目标/端止限幅壁，仅在 `delta_cz == 0` 时退回纯墙体引导，兼顾带相对步长的追焦（`S3`）与零相对量墙体探索（`verify_v216` Gate 3D）。

### 2.3 相平面硬件 `vsqrt.f32` 减速包络与制动外推界约束

- **相平面开方减速包络（`eval_fam2_approach`）**：当 `fam2_ext != 0` 且预测残差位于 $(\mathrm{step\_vd},\,3\,\mathrm{step\_vd})$ 减速区时，采用 Cortex-M4F 单周期硬件浮点开方 `boryoza_sqrtf`（`vsqrt.f32`）生成在边界 $3\,\mathrm{step\_vd}$ 处 $C^0$ 连续、向下渐缩至约 $0.58\,\mathrm{step\_vd}$ 的匀减速包络：
  $$v_{\mathrm{env}} = \sqrt{\frac{1}{3}\,\mathrm{step\_vd}\cdot |\mathrm{pred\_err}|}$$
  并在进入 $|\mathrm{pred\_err}| \le \mathrm{step\_vd}$ 时直接转入单帧 `STEP_ACT_ABS` 精准着陆，同步写入 `servo->active_step_se`。
- **热线外推制动与 32 位定点数时间差**：`interpolate_hotline_cz` 先将 `dt_sample` 与 `dt_now` 钳位入 `uint32_t` 再转 `float`（彻底消除 `__aeabi_ul2f` 软浮点库调用）；在 `af_stop_req != 0 || e0_brake_rem > 0` 制动窗将 `dt_now` 减半，并保证 `|pred_se - focus_pos_se| <= active_step_se`，消除盲搜急停与高速切换末端热线过冲。

---

## 3. 保守 Cortex-M4F MCU 性能优化清单（零功能语义改变）

| 优化点 | 目标文件与函数 | 消除的底层开销 | 保守边界 |
|---|---|---|---|
| **O1：消除 64 位除法库调用** | [vr_duty_controller.c](../../se-nz/src/02_core/vr_duty_controller.c) (`vr_controller_step`) | 差值超过 `65535 * VR_QUARTER_MS_TICKS` 直接饱和并同步时间戳，其余走硬件 32 位 `udiv`，彻底移除 `__aeabi_uldivmod` / `__udivmoddi4` | `remaining_250us` / `auxiliary_250us` 位宽本就是 `uint16_t`，数学等价 |
| **O2：消除 64 位转单精度浮点** | [af_servo_bridge.c](../../se-nz/src/02_core/af_servo_bridge.c) (`interpolate_hotline_cz`) | `dt_sample` / `dt_now` 钳位至 `<= 300,000` 的 `uint32_t` 后使用硬件 `vcvt.f32.u32`，移除 `__aeabi_ul2f` | `dt_now` 本就受 `EXTRAP_MAX_TICKS` 钳位，`dt_sample` 超限直接走恒速保护 |
| **O3：`union` 替代 `memcpy` 浮点双关** | [optics_xlate_fpu.c](../../se-nz/src/02_core/optics_xlate_fpu.c) (`boryoza_isfinite_f32` / `log2f` / `exp2f` / `write_f32_le`) | `-ffreestanding` 下编译器对 `memcpy(&u32, &f32, 4)` 发出 `bl memcpy` 跳帧调用；改用 `union { float f; uint32_t u; }` 编译为单条 `vmov r0, s0` | 严格局限在 `02_core/`（Hard-FPU 域），C11/C17 合法联合体双关 |
| **O4：静态快照 `CMD 10/11/12` 脏缓存** | [z_snapshot_builder.c](../../se-nz/src/02_core/z_snapshot_builder.c) (`z_snapshot_publish`) | 每 VSync 双缓冲切换时对比静态字段签名与 `c11_identity_hash`（`vendor_tag_5b` + `lens_name_ascii` FNV-1a），仅在身份/名称/焦距/光圈边界/VR 能力改变时重编相应快照 | 双银行独立维护 `done_mask`，`CMD 0x3F` 写入目录外新镜头名时立即全量重编双 bank `CMD 0x11` |
| **O5：`ID05` 光学曲线按变化门控拷贝** | [lens_session.c](../../se-nz/src/02_core/lens_session.c) (`lens_session_unpack_id05` / `decode_id05_groups`) | 在 `curve_frames_seen >= 2` 后仅当光学组 `[5,11,12,13,15,16,17]` 实际改变时执行曲线拷贝与 `decode_typed_curve_slots`，稳态仅更新动态组 | 初始连续 2 帧即使相同仍放行成熟计数保证 `optics_ready` 无延迟，后续同帧跳过 |
| **O6：校验和 4× 展开** | [lens_session.c](../../se-nz/src/02_core/lens_session.c) (`boryoza_neg_sum8` / `boryoza_sum16_emount`) | 按 4 字节步进累加循环展开降低高频 SPI3/UART 封包校验分支开销 | 纯整数算术，严格零 VFP |

---

## 4. `V2.18` 最终实测对比与回归结果

### 4.1 固件产物与静态安全门禁（`pack_manifest.json`）

- **`V2.18`（`LF 2.18` / `.readme_ver = 12020b1a`）**：
  - `BORYOZA_V218.PUB`：`158,720 B`（`310` 个 `512 B` 扇区，原始镜像 `158,284 B`，`program_end = 0x00036BF0`，较原厂 `84A8` `217,600 B` 缩减 `27.06%`），`SHA-256 = 9a8a515b7f45199d1325eb9175ce8819c37409119b5567b0373950b6c5c84b58`
  - `BORYOZA_V200.bin`：`SHA-256 = 5e15e16e4b762db3be0bc3e6531e493131ad9814a1eed8741286ce63555205dc`
  - 保留基线 `BORYOZA_V217.PUB`（`LF 2.17` / `11020b1a`）：`SHA-256 = 32df26fde8dc3bafca1bcdc2c46009670b8334da8b58da4fc45aab8d7417941d`
  - 全链路静态安全：`nofpu_violations = 0`（`85` 函数）、`closure_violations = 0`（`32` RAM ISR + `5` EFM SRAM 闭包）、`check_source_comments.py` `24` 文件 `0` 违规、`syntax_check.py` `82/82` 通过、`lint_check.py` 通过、`qa_oracles.py freshness` 通过。
  - Z8 机身版本号同步：Z8 `3.10` `CMD 0x10` 返回 `0x00001802` / `prop05 = 0x02180000`；Z8 `3.12` bank `0x0B[08..0B]` 返回 `00001802`。

### 4.2 7 场景 Q8-mm 闭环基准三方对比（`v218_vs_etz21_benchmark.json`）

| 场景 | `ETZ21 0207`（`1/7`） | `BORYOZA V2.17`（`2/7`） | `BORYOZA V2.18`（`7/7`） | `V2.18` 核心指标 |
|---|---|---|---|---|
| **S1 `150mm F2` Inf -> 0.75m** | 未锁绿（`8573/11400`, `15712.7 um`） | 确认蠕变超时 | **12 帧锁绿** | `err = 0 SE` / `0.0 um`，`c0 = -1` |
| **S2 `150mm F2` 0.75m -> Inf** | 未锁绿（`+51.2mm` 极点反转撞至 `13000`） | 15 帧锁绿 | **12 帧锁绿** | `err = 0 SE` / `0.0 um`，`c0 = -1` |
| **S3 `150mm F2` `E4` `delta_cz` 追焦** | 未锁绿（`13000/7500`, 过冲 `30993.9 um`） | 过冲 `29357.8 um` 失败 | **16 帧锁绿** | `err = 0 SE` / `0.0 um`，`overshoot = 65.3 um` |
| **S4 `150mm F2` 飞行中换目标** | 未锁绿（`12626/5200`, 过冲 `39720.1 um`） | 10 帧锁绿 | **8 帧锁绿** | `err = 0 SE` / `0.0 um`，`0.0 um` 过冲，`0` 次重发 |
| **S5 `50mm F2` Inf -> MFD 0.40m** | 未锁绿（`9192/12400`, `6921.0 um`） | 确认蠕变超时 | **12 帧锁绿** | `final = 12387/12400`（`30.0 um`），`c0 = -3` |
| **S6 `150mm F2` 盲搜 `C0` + `E0/E2`** | 未锁绿（`13000/9800`, `18895.3 um`） | `44.1 um` 残差超焦深 | **16 帧锁绿** | `err = 0 SE` / `0.0 um`，`overshoot = 66.1 um` |
| **S7 `0xC56F` 双浮动细确认 `18 SE`** | 6 帧锁绿（`err = 0 SE`） | `40 SE` 吞指令锁死 | **6 帧锁绿** | `final = 6997/7000`（`3 SE` / `20.7 um`） |

### 4.3 Cortex-M4F MCU 优化实测指标与独立 Break-It 闭环

- **ELF 静态消减（`O1..O3`）**：`BORYOZA_V200.elf` 中 `udivmoddi4`/`uldivmod`/`ldivmod` 符号 `0` 个、`bl __aeabi_ul2f` 调用 `0` 处、`boryoza_exp2f` 与 `boryoza_log2f` 反汇编 `bl memcpy` 调用 `0` 处、硬件浮点开方 `vsqrt.f32` `1` 处（`eval_fam2_approach` 实测相对步长平滑递减 `[240, 205, 169]`）。
- **热点隔离对比（`O4..O5`，`SteadyOpticsPlant` 10 帧 VSync）**：
  - `decode_id05_curves`：稳态命中 **`0` 次（`0` 条指令）** vs 冷重建 `14` 次（`2,114` 条指令）；
  - `z_snapshot_publish`：稳态 **`11,102` 条指令** vs 冷重建 `11,766` 条指令。
  - 注：`HC32FullChipSim` 全窗口 `step_quanta` 无 `WFI` 休眠，10 帧固定空转 `5,250,000` 次主循环轮询并受 `3,000` 指令 SPI3 轮询量子对齐影响（全窗口计数 `V2.17 = 5,403,703`、`V2.18 = 5,403,709`），因此性能验证以确定性 ELF 指令审计与函数级热点计数为准。
- **Break-It 审计闭环（子代理 `955b9537`）**：4 项 BLOCKER（`FAM2_DECEL_GAIN = 1/3` 连续开方减速、`ID06/ID05` 乱序与近摄恒 `k_se` 防假锁、`F2.0` 光圈独立 `c0_safe` 与 `1+beta+beta^2/3` 积分安全系数 + `8mm` 广角支持、`CMD 0x11` FNV-1a 身份哈希防 `CMD 0x3F` 双 bank 脱同步）及关键 WARN（`E_CMD_ID3F` 激活区处理、`0x3E` 光圈应答补刷 `K_Z`、`active_step_se`/`hotline_seen` 停稳与掉电清零、`curve_frames_seen < 2` 初始同帧成熟、benchmark 消除重复遥测）全部完成编码并由 `Gate 4`（`5/5`）实证通过。

### 4.4 全量 12 项门禁与历史回归矩阵

- `verify_v218_defocus_and_mcu_opt.py`：**`5/5 PASS`**（`v218_defocus_and_mcu_opt_report.json`）
- `benchmark_defocus_closed_loop_etz21.py`：**`7/7 PASS`**（vs `ETZ21 1/7`、`V2.17 2/7`）
- `test_m0_bl_and_mcu_sim.py`：**`78/78 PASS`**
- `test_m1_core_z8_closed_loop.py`：**`53/53 PASS`**
- `test_vr_d3_transactions.py`：**`162/162 PASS`**
- `test_photo_lifecycle.py`：**`28/28 PASS`**
- `verify_v208_hardware_truth.py`：**`16/16 PASS`**
- `verify_v212_gm50150_optics.py`：**`6/6 PASS`**
- `verify_v215_z8_closed_loop_large_defocus.py`：**`5/5 PASS`**
- `verify_v216_gm50150_close_lock_and_lf_ver.py`：**`4/4 PASS`**
- `verify_v217_gm50150_confirm_green_and_sweep.py`（对 `BORYOZA_V217.PUB`）：**`4/4 PASS`**
- [flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) `--selftest`：**`PASS`**（`expected_version = V2.18`, `158,720 B`）

---

## 5. `BORYOZA V2.19`（`LF 2.19` / `13020b1a`）综合演进、压缩编码与全芯片验证

### 5.1 两遍式幅值类别 Canonical Huffman 光学表压缩与无损光谱编码

- **Canonical Huffman 与定点三维反演解码**（`gen_lens_db.py` / [optics_xlate_fpu.c](../../se-nz/src/02_core/optics_xlate_fpu.c)）：
  - 针对 `CMD 0x24 A`（`GRID_T3/T4 <= 1.0 × U5 × V3`，`QA = 15`）与 `CMD 0x24 B`（`GRID_TB3 × U6 × V3`，`QB = 20`），首先利用对称性识别 `u_const` 径向恒定槽位、抽取 56-bit 稀疏掩码（`misc_mask`, `dist_mask`）并结合光圈坐标执行 `flag == 3` 二阶 `int32_t` 差值预测；
  - 将全部预测残差映射到幅值类别符号（`g_opt_huff_counts[12]` + `g_opt_huff_syms[29]`，共 `41 B` 且最长码字 `max_len <= 11`），由 C 解码端经 `INV_T3_Q22` / `INV_T4_Q22` / `INV_U5_Q22` / `INV_V3_Q22` / `INV_TB3_Q22` / `INV_U6_Q22` 定点反演恢复 IEEE-754 单精度多项式；对于 `GM50150`（`0x807B`）与 `Sigma 150-600`（`0xC56E`），启用 `exact_q12`（`0x80` 标志位）实现 `CMD 0x24 A` 逐字节无损编码。
- **实测 Flash 压缩与保真指标**：
  - `g_tamron_z_optics_db[13]`：`cmd24_a_packed` 合计 `34,492 B` + `cmd24_b_packed` 合计 `15,606 B` = **`50,098 B`**（对比原始 `13 × (7,426 + 3,468) = 141,622 B`，**缩减 `91,524 B` / `64.6%`**）；`12` 组 Tamron 验证档案最大暗角偏差 **`0.3662%`**、最大 `LCA Q20` 残差 **`22`**；`GM50150` `CMD 0x24 A` **逐字节精确还原**、`LCA Q20 = 10`；
  - `g_optical_primary_a_packed`（`Sigma 150-600`）：**`1,638 B`**（较 `7,426 B` 节省 `5,788 B`），`CMD 0x24 A` **逐字节精确还原**；
  - `g_spectral_packed_blob`（**`2,780 B`**）+ `g_spectral_packed_offs[58]`（`116 B`）= **`2,896 B`**：`57` 条真实光谱透射率曲线采用无损 delta-nibble 打包并全量精确解码（较原始 `57 × 81 = 4,617 B` 节省 `1,721 B`）。

### 5.2 伺服控制、光学元数据与容错回退升级

- **`CMD 0xEE` 20fps 高速连拍单权威**：`handle_z_cmd_ee` 直接将主目标 `raw_t1` 写入 `target_cz`（零桥接速度前馈外推），按同侧墙体 `mosi[20..23]` 限幅并支持 `mosi[25] & 0x06` 即时制动；`fam2_ext` 在 `E4 / EE` 下直接反向 `(0x1D, 0x00)`。
- **Sony Focus-by-Wire 电子齐焦**：空闲或机身侧 MF 变焦跨越触发 `ID04[9] = 0x81` 并于停稳 `2` VSync 后下发单次 `1D` 微修；物理拧动对焦环即时更新焦点锚点并维持纯手动随动。
- **`DS13` 呼吸焦距与光学校正扩展**：`z_eff_focal_q4 / 16.0f` 传入 `update_pole_safe_c0` 精确收敛近摄呼吸效应，同时 `f_cur_mm` 保持标称焦距供 EXIF/HUD；全系支持手动光圈环 `cur_stopdown_delta`、`10 m` 平滑物距过渡、Sony `1.4x/2.0x` TC（`0x8130..0x8148`、`CMD 0x10[0x19..0x1A] = 00 00`）与双端 MFD 线性插值。
- **初始化后校正断开与行为式距离回退**：初始化完成后按镜头类型锁固/断开校正通道并保留实时 `delta_av`；纯按运行态行为检测冻结或异常 `dist_code_x`，切换至 3 轴防抖 + 镜身 OIS 与 `ID06` 闭环 AF 并向 Z8 上报 `(-1, -1)`，在镜头固件修复编码器后自动恢复全量功能。

### 5.3 HC32F460 SoC 加速、`< 105 ms` 启动与 ELF 资源快照

- **SoC 级性能优化**：启用 `EFM_FRMC = 0x00010050`（192 MHz 5-wait 叠加大容量 I/D Cache，在 EFM 编程/擦除前后以 `CRST` 安全失效）；`RAMFUNC` 四字展开 `memcpy/memset/strlen`；`.rodata` 256 项半字节 CRC32 查表；`CORE_OPT = -O3` + `NOFPU_FLAGS = -mgeneral-regs-only`（`89` 函数 `0` FPU 违规，`36` RAM ISR + `5` EFM SRAM 闭包 `0` 违规）；16 桶 `g_lens_bucket_bounds` 加速 `lens_db_lookup`；`CMD 08` 后早释 `PA9`（`Sigma 150-600` 首窗就绪 **`101.89 ms < 105 ms`**、空环 `30.4 ms`、`SCK 80ms` 延迟 `86.4 ms`）叠加同镜头 `sony_ds13_lid` 热重启 `DS13` 缓存复用。
- **产物指纹（`pack_manifest.json`）**：
  - `BORYOZA_V219.PUB`：**`163,328 B`**（`319` 扇区，原始镜像 `163,248 B`，`program_end = 0x00037DF0`，`.readme_ver = 13 02 0b 1a` = `LF 2.19`），`pub_sha256 = a3ca6fe5a3eb98949672bdd429668483b1c2980535d74feb2d505560f65a0ebd`，`bin_sha256 = 4ade2d5ef03b1f7e26d4ef81621814764f0af2a9b5ccf72a3157a3e67ada7a81`。
  - ELF 段占用（`.text = 156,556 B`、`.isr_vector = 4,096 B`、`.ramfunc = 2,580 B` 止于 `0x1FFF8E14`、`.dma_sramh = 9,828 B`、`.bss = 5,080 B`、`.optics_sram2 = 63,564 B`、`.diag_sram3 = 16,384 B`、`.retram_noload = 28 B`）。
  - Z8 机身识别：`z8_310_prop05_hex = 0x02190000`，`z8_312_bank0b_hex = 00001902`；保留基线 `V217`（`32df26...`）与 `V218`（`9a8a51...`）字节完全不变。

### 5.4 `V2.19` 全量 16 项自动化验证与回归汇总

| 验证入口 / 构建目标 | 结果 | 通过 / 总数 | 关键实测指标 |
|---|---|---|---|
| `verify_v219_features.py` `--selftest` | `PASS` | `8/8 + selftest` | `PA9=101.89ms`, 57 光谱无损, 12 组 Huffman, Blackbox 扇区滚动擦写保留末 8 条 |
| `verify_v217_gm50150_confirm_green_and_sweep.py` (`V217`) | `PASS` | `4/4` | 基线 SHA `32df26fde...` |
| `verify_v218_defocus_and_mcu_opt.py` (`V218`) | `PASS` | `5/5` | 基线 SHA `9a8a515b7...` |
| `verify_v213_field_fixes.py` `--pub build/BORYOZA_V219.PUB` | `PASS` | `4/4` | E0/E4 开放墙扫、F2 确认、热线状态同步 |
| `verify_v212_gm50150_optics.py` `--pub build/BORYOZA_V219.PUB --selftest` | `PASS` | `6/6 + selftest` | `GM50150` `CMD24A/B` 与动态倍率边界 |
| `test_af_feedback_behavior.py` | `PASS` | `17/17` | 多延迟执行器闭环收敛与反例拒绝 |
| `test_m0_bl_and_mcu_sim.py` | `PASS` | `78/78` | 全芯片 `HC32FullChipSim` + BL + Blackbox 事件解码 |
| `test_m1_core_z8_closed_loop.py` | `PASS` | `53/53` | ASan+UBSan 闭环、热限速、DS13 呼吸、VR |
| `test_z8_native_recognition_sm.py` | `PASS` | `9/9` | Z8 AArch64 识别状态机 + 164 描述符清扫 |
| `test_brand_feature_chains.py` | `PARTIAL` | `565/565` | 18 型号全协议链通过；实镜台架输入独立跟踪 |
| `test_stress_protocol_matrix.py` `--cycles 2 --selftest` | `PASS` | `35,753/35,753` | 矩阵、负例、预握手、25,704 描述符 |
| `test_stress_hardware.py` `--selftest` | `PASS` | `19/19 + selftest` | 热插拔抖动、睡眠唤醒、丢帧恢复 |
| `test_id05_selectors.py` | `PASS` | `48/48` | `ID05` 轮转选择器路由 |
| [flash_usb_bor.py](../../se-nz/tools/flash_usb_bor.py) `--selftest` | `PASS` | `1/1` | `expected_version = V2.19`, `163,328 B` |
| [decode_blackbox.py](../../se-nz/tools/decode_blackbox.py) `--selftest` | `PASS` | `6/6` | `32 B` `arg3` 格式与新事件码解码 |
| [check_source_comments.py](../../se-nz/tools/check_source_comments.py) `--selftest` | `PASS` | `24/24` | `violations = 0` |
