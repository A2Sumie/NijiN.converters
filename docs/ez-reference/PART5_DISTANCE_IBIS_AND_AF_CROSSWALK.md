# Part 5：Sony E / Nikon Z 全链路距离字段、五轴防抖、EXIF、MF 标尺与 AF 控制对照参考

本文系统梳理 Sony E 卡口与 Nikon Z 卡口在对焦距离、光学放大倍率、马达编码器轴位、凸轮标定表、五轴机身防抖（IBIS / SteadyShot）、EXIF 写入、取景器 MF 标尺以及自动对焦（AF）闭环控制上的协议分工与差异，并与 `SE-NZ` 桥接固件实现逐一对应。

前置与关联卷目：[`PART1_NIKKOR_Z_FLEET.md`](PART1_NIKKOR_Z_FLEET.md)、[`PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md`](PART2_Z8_AF_AND_EMOUNT_PROTOCOL.md)、[`PART3_EZ_BRIDGE_AND_ERRATA.md`](PART3_EZ_BRIDGE_AND_ERRATA.md)、[`PART4_PDAF_MATH_AND_CONTROL_OPT.md`](PART4_PDAF_MATH_AND_CONTROL_OPT.md)、[`Z8_SONY_AF_ARCHITECTURE_STUDY.md`](Z8_SONY_AF_ARCHITECTURE_STUDY.md)、[`HC32F460_SE_NZ_DESIGN_SPEC.md`](HC32F460_SE_NZ_DESIGN_SPEC.md)。

---

## 1. Sony E 卡口全部距离、倍率、轴位与标定表字段（四层通道）

Sony E 卡口并不把“对焦距离”浓缩成单一数字，而是拆在 **4 条并行协议通道** 上分别服务自动对焦、光学渲染、五轴防抖与 EXIF 记录（解析实现见 [`lens_session.c`](../../se-nz/src/02_core/lens_session.c) 与 [`boryoza_types.h`](../../se-nz/include/boryoza_types.h)）：

### 1.1 通道一：`ID 0x05` 周期光学遥测帧（L2B，每 VSync 一包，18 组动态掩码）

`ID 0x05` 按 `CMD 0x0A` 协商出的 `active_mask05` 依次拼接激活组，各组长度表为：
`GRP05_LEN[18] = {8, 1, 8, 3, 10, 28, 2, 6, 1, 2, 8, 6, 6, 6, 1, 6, 3, 3}`（`wire[6]` 起解码）。核心与距离、对焦及光学状态相关的组分布如下：

| 组号 | 长度 | 字段偏移与宽度 | 编码公式 / 哨兵语义 |
|---|---:|---|---|
| **`grp[4]`** | `10 B` | `+0..1` (`u16_le`) `dist_code_x` | **直接光学物距对数码**。`0xFFFF`（及 `>= 0x0700` 光学无穷远阈值）为无穷远；有限物距 $D_{\mathrm{m}}$ 满足 $x = 384 + 64\log_2(D_{\mathrm{m}})$（`1.00 m -> 384`，`0.25 m -> 256`，哨兵假值 `0x00FF / 255` 若按式硬算约合 `0.247 m`） |
| **`grp[4]`** | `10 B` | `+3` (`u8`) `mag_code` | **横向放大倍率对数码**。`0x00..0xFE` 为有效倍率：$\beta = 2^{5 - \mathrm{code}/16.0}$（如 `0x50 = 80` $\Rightarrow$ $\beta = 1.0$ `1:1` 微距）；**`0xFF`（`255`）为“无效 / 未上报”哨兵** |
| **`grp[4]`** | `10 B` | `+4..5` (`u16_le`) `f_opt_0p1mm`<br>`+6..7` (`u16_le`) `f_nom_0p1mm` | 分别携带**受对焦呼吸修正的实时有效焦距**与**标称焦距**（单位 `0.1 mm`） |
| **`grp[0]`** | `8 B` | `+0..1` (`u16_le`) `P_cur` | 当前光圈编码（`256 steps/EV`；全开/最小光圈由 `TX08` 基准叠加 `grp[5] Delta_AV` 结算） |
| **`grp[2]`** | `8 B` | `+0..1` (`u16_le`) `exit_pupil_raw` | 随变焦/对焦动态变化的出瞳距离编码 |
| **`grp[3]`** | `3 B` | `+0..1` (`u16_le`) `P_ring`<br>`+2.bit0` `manual_ring_active` | 镜身物理光圈环刻度与手动 / `A` 档切换状态 |
| **`grp[5]`** | `28 B` | `+0..1` (`s16_le`) `Delta_AV` 等 | 近摄有效光圈衰减量 `Delta_AV`（T 值/近摄光圈修正）与暗角/畸变 6 B 核心节点 |
| **`grp[7]`** | `6 B` | `+0` (`s8`) `mf_ring_dir`<br>`+1` (`u8`) `mf_ring_step`<br>`+2.bit1` `is_mf_switch`<br>`+4.bit3` `l_fn_pressed`<br>`+4.bits[6:4]` OIS 开关/模式 | 手动对焦环旋转方向/步长脉冲、`AF/MF` 物理开关（`0=AF, 1=MF`）、`AFL / Focus Set` 键与防抖开关状态。机身据此激活 EVF MF 距离标尺、DMF 介入与对焦放大 |
| **`grp[9]`** | `2 B` | `+0.bits[1:0]` `ois_active_fb` | 镜身光学防抖实时工作反馈 |
| **`grp[11..13, 15..17]`** | `3×6+6+3+3 B` | 色差选择器与边缘扩展节点 | 随变焦/对焦动态更新的像差补偿节点 |

### 1.2 通道二：`ID 0x06` 周期执行器反馈帧（L2B，每 VSync 一包，机械/编码器轴位）

在 `A1 II` / `A9 II` 机身日志中对应 `fp_info2` 与 `Pos_Cycle`（每 VD 上报 N 个位置采样），早期公开逻辑分析仪抓包资料亦将同源周期状态流记作 `0x30` 帧：

- **`focus_pos_se`**（标准位置在 `payload[2..3]`；适马 `family=1 && proto > 0x0110` 与 `Zeiss Batis 40` 在 `payload[20..21]` 携带实时采样位，长度 `>= 22 B`）：对焦马达编码器实时轴位计数 $S_E$。
  - **带符号差异**：Sony 原厂、Sigma 等使用有符号 16-bit（`s16_le`）；Tamron 与早期 `AF_CLASS_B_LEGACY_F1_F2` 镜头使用无符号 16-bit（`u16_le`，会无回卷跨越 `32767 -> 32768 -> 40000`）。
- **`lim_far_se`（`payload[7..8]`）与 `lim_near_se`（`payload[9..10]`）**：当前焦段（及电子限位状态下）的**无穷远端与最近对焦端行程边界**。
- **`zoom_norm`（`payload[11..12]`）**：归一化变焦编码器位置（`0x0100..0x3F00`）。
- **`defocus_k_se`（`payload[13..14]`）**：实时离焦灵敏度因子。另外扩展格式支持 **`ID 0x06 tag 0x44`**（直接报无关 F 值的纯几何灵敏度 $K_{\text{ext}} = 2\,\mathrm{d}S_E/\mathrm{d}z_{\mathrm{mm}}$）。
- **`temp_celsius`（`payload[15]`）**、`hw_moving`（`payload[1] & 0x07`）与 `mf_ring_byte`（`payload[1] == 1 || 2`）。
- **早期/社区 `0x30` 预缩放背景**：部分早期抓包文献在周期帧 `b31..32` 观测到无穷远码值满足 $x_\infty \approx 115 \cdot \sqrt{f_{\mathrm{mm}}} \cdot k_{\mathrm{lens}}$（随 $\sqrt{f}$ 变化的镜头侧预缩放轴位码），机身再结合凸轮系数把码差还原为对焦组物理位移。

### 1.3 通道三：`CMD 0x3F` + `CMD 0x3A / 0x3B` 上电静态分片表（`Camera::Lens::DividedInfo`）

在 `A1 II` `cpapp-b.bin`（LIF 模块装入基址 `0x0014C000`，RTTI 区 `0xC79290..0xC793D0`）中，挂载握手阶段先以 `CMD 0x3F` 读出支持位图（如腾龙须在 `session_mode == 2` 时调用），再由 `CMD 0x3A / 0x3B` 通过共用的分包信封（请求结构 `0x18 B`，含 `+0x04 CommandId`、`+0x07` 分段号、`+0x14 = 0xDEADBEEF` 调试哨兵、`1024 B` 净荷缓冲及 `VD_Counter` VSync 分槽队列）分页拉取 6 个净荷子类：

1. **`DividedInfoCamcurveData`（`Mount_FcZm_CamCurveData.cpp`）**：镜头上报的 **per-lens 机械标定表**，核心为 **`fc_pitch`**（编码器每计数对应对焦组物理行程，典型 `1–10 µm/count`）与两段带校验的 `(zoom, focus)` 二维凸轮曲线表（经 `1st/2nd` 双段回显比对验证）。
2. **`DividedInfoBreathingCorrectionData`（`dataset 13` / `0x0D`，长 `3,870 B` 或 `30 B`）**：按对焦距离分布的呼吸效应视角修正表。
3. **`DividedInfoSteadyShotInfo`（`Mount_DividedInfo_SteadyShotInfo.cpp`）**：防抖光学协同信息分包通道（回调 `CallBack_SteadyShotInfo_Com_1st @ 0x460174` / `_Com_2nd @ 0x460660`，状态机 `INIT -> STANBY`）。
4. **`DividedInfoEclipseCom`（`dataset 0`）**：原厂片内光学暗角/畸变/色差静态档案。
5. **`DividedInfoFnoCorrectAbility`**：对焦伸出量引起的有效光圈/F 数修正能力。
6. **`DividedInfoIrisDrivabilityData`**：光圈叶片机械驱动能力曲线。

### 1.4 通道四：全局能力与线性手动环命令（`CMD 0x08 / 0x09 / 0x22`）

- **`CMD 0x08`（`CoC = 3072 / 0x0C00`）与 `CMD 0x09`（`CoC = 3000 / 0x0BB8`）**：机身向镜头注入参考弥散圆基准，激活部分镜头（如 Sigma HLA 全系）的实时 `defocus_k_se` 计算；回传 `202 B TX08` 含焦距、光圈跨度、OIS 与高刷能力位。
- **`CMD 0x22`**：机身下发的线性手动对焦（Linear MF）旋转行程/编码设置。

---

## 2. Sony 原厂机身三大消费端（五轴 SteadyShot、DCV 反卷积、ARW EXIF）

实证源自 Sony `A9 II` `AV-CAM.BIN` / `BONOBO.BIN` 及 `A1 II` `cpapp-b.bin`：索尼机身将上述通道按职责严格拆给三个下游消费者，**一级源、三级精度、各取所需**。

### 2.1 消费端一：五轴 SteadyShot 机身防抖（吃“轴位 + 凸轮标定表”）

五轴防抖包含 **3 轴角度（Pitch 俯仰 / Yaw 偏航 / Roll 滚转）** 与 **2 轴平移（X / Y Shift）**：
- 角度旋转 $\theta$ 在传感器像面产生的位移为 $v \cdot \theta = f(1 + \beta)\theta$（其中 $f$ 为焦距、$\beta$ 为横向放大倍率）；
- 无穷远时 $\beta \to 0$，仅需 **焦距 $f$**；近摄至 `1:1`（$\beta = 1$）时 $(1+\beta)=2$，**补偿量翻倍**，X/Y 平移轴的增益与精度直接取决于对焦延伸量。

在索尼原厂架构中，主核（`AV-CAM` / `Cetus CP`）承担旋转姿态与光学参数解算，防抖执行器（`AGUS`）由独立协处理器 **`Bonobo`** 固件运行双陀螺 + Hall 传感器 2-DOF PID 闭环（`GyroStart / GyroStop` + `gyro_sync_id` + `GyroSamplingDelayUsec` 将镜头陀螺与机身陀螺时间对齐，汇入同一融合环）。薄透镜下 $\beta = e / f$，其中对焦组物理伸出量 $e = (x - x_\infty)\cdot p$，因此平动像移直接写成**无需显式经过 `mag_code` 的仿射式**：

$$\mathrm{shift} = f \cdot \theta + (x - x_\infty) \cdot p \cdot \theta$$

其中：
- $x$ 来自 **`ID 0x06 focus_pos_se`**（`fp_info2` / `Pos_Cycle`）；
- $x_\infty$ 来自挂镜与实时焦段锚点（`us_B4Mount_FocusPos`）；
- $p = \mathrm{fc\_pitch}$ 来自上电 **`CMD 0x3A / 0x3B DividedInfoCamcurveData`**；
- $f$ 按静止/视频拆轨下发为 **`us_steadyshot_focal_length_still / _movie`**，并配合 `DividedInfoSteadyShotInfo`。

因此：**索尼原厂机身五轴防抖不依赖 `ID 0x05 grp[4]` 的 `dist_code_x` 或 `mag_code`**。即使镜头把 `ID 0x05` 距离与倍率全填 `255`，只要正确上报 `ID 0x06` 轴位并上传了 `CamCurveData / SteadyShotInfo`，五轴平移仍可正常计算；若镜头完全省略 `CamCurveData / SteadyShotInfo`，机身则仅保留按焦距工作的 **3 轴角度防抖**。

### 2.2 消费端二：DCV 机内去卷积锐化（吃 `focus_pos` 粗分桶）

`AV-CAM` `job_YC_SetPreParam` / `job_YC_SetSaCamDspParam` 直接传入十六进制 `focus_pos`，在多套离散点扩散函数表（`dcv_psf_table_{macro_still, macro_movie, still, movie, ...}.bin`）间做 **微距 / 常规分桶切换**，只要求局部区间粗精度。

### 2.3 消费端三：ARW / JPEG EXIF 距离标签（吃 `ID 0x05 grp[4]+3` `mag_code`）

索尼相机写入照片 MakerNote 的 **`Sony2Fp.FocusPosition2`**，字节级直接取自 **`ID 0x05 grp[4]+3`（`u8 mag_code`）**。EXIF 读取器（如 `ExifTool`、`Darktable` / `Exiv2`）再结合 35mm 等效焦距按薄透镜近似公式反推显示物距：

$$D \approx \left(2^{\mathrm{FocusPosition2}/16 - 5} + 1\right) \cdot f$$

- 当 `mag_code` 在 `0..254 (0x00..0xFE)` 内有效时，照片可反算出连续物距；
- 一旦镜头在 `grp[4]+3` 填入协议默认无效值 **`0xFF`（`255`）**，`FocusPosition2` 就固定记录为 **`255`**，照片 EXIF 距离显示缺失或锁死在 `255`。

---

## 3. 唯卓仕等副厂镜头“EXIF 卡在 `255`，但 MF 时能显示对焦距离”的底层原因

这种现象在副厂 E 口镜头上很常见，看似矛盾，实则是**“屏幕 MF 距离指示”与“成片 EXIF 记录”使用了不同的协议槽位**，通常由以下三条独立机制之一或叠加造成：

### 3.1 机制一：`dist_code_x`（物距码）与 `mag_code`（倍率码 = `255`）解耦

部分副厂 E 口镜头固件在 `ID 0x05 grp[4]+0..1`（`dist_code_x`）里维护了实时物距查表，却把 **`grp[4]+3`（`mag_code`）固定填为 `0xFF`（`255`）**。
- 在索尼机身上：切换 MF 拧对焦环时，取景器 UI 可以从 `dist_code_x`（或凸轮比例）显示变化的对焦距离；但拍出来的 ARW 照片里，`Sony2Fp.FocusPosition2` 抄的是 `mag_code`，所以 EXIF 永远卡在 **`255`**。

### 3.2 机制二：取景器 MF 距离进度条只需 `ID 0x06` 马达轴位就能动

无论在索尼原厂机身还是尼康 Z 机身（经转接环），EVF 底部常见的 **“$\infty \leftrightarrow \text{近端}$ MF 距离标尺游标”** 与“精确米制数字/EXIF”是解耦的：
- 即使一支镜头把 **`dist_code_x`（`0x00FF`）** 和 **`mag_code`（`0xFF`）** 全都填死在 `255`，只要它的自动对焦马达正常工作，其 **`ID 0x06 focus_pos_se` 和 `[lim_far_se, lim_near_se]` 就是真实平滑变化的**；
- 切入 MF 并转动对焦环（触发 `ID 0x05 grp[7]` / `ID 0x06[1]`）时，机身（或转接环）只要根据归一化行程比例 $(S_E - S_{E,\infty}) / (S_{E,\mathrm{near}} - S_{E,\infty})$ 就能画出会动的距离标尺。用户看到屏幕弹出距离条且游标随拧环移动，就会反馈“MF 时能显示对焦距离”。

### 3.3 机制三：部分镜头固件内部存在 `AF` 与 `MF` 距离计算分支差异

部分第三方 STM/VCM 镜头固件在高频 `AF` 循环期间不计算物距查表（固定将 `dist_code_x` 设为 `0x00FF = 255`），仅在进入 **手动对焦（`MF`）任务 / 对焦环中断** 时（例如带镜身 OLED 距离屏的型号）才主动查表刷新 `dist_code_x`。此时：**AF 拍摄的照片没有有效物距（或卡在 `255` 对应的 `~0.25 m` 假值），而切到 MF 拧环就能看到真实动态距离**。

### 3.4 可观测性判定：只看到“有 MF 距离、无 EXIF 距离”，能反推有没有放大倍率码吗？

结论取决于运行在哪个卡口机身上：

| 运行平台 | 能否根据“有 MF 距离、没 EXIF 距离”反推是否有倍率/凸轮信息？ | 协议层依据 |
|---|---|---|
| **索尼原厂机身** | **能确认没有 `ID 0x05 mag_code`；无法判断有没有 `CamCurveData`** | 索尼 ARW `Sony2Fp.FocusPosition2` 一比一镜像 `grp[4]+3` `mag_code`，EXIF 为 `255` 即实证 `mag_code == 0xFF`。但索尼机身五轴平移防抖消费的是 `CMD 0x3A/0x3B CamCurveData (fc_pitch)`，该表不写入 EXIF，无法从 EXIF 反推 |
| **尼康 Z 机身 + 转接环** | **无法仅凭 EXIF 反推镜头是否上报了 `mag_code`** | 尼康 NEF `FocusDistance` 来自 `Property 0x117`（源自 `dist_code_x`），而 Z 五轴平移吃的是 `Property 0xA3`（源自 `mag_code` 或转接环目录/物距补算），二者完全分离且 `A3` 不直接写入标准 EXIF 距离标签 |

**零逻辑分析仪快速验明方法**：将镜头装在任意索尼 E 口机身上，分别用 `AF` 和 `MF` 近摄对焦各拍一张照片，执行：

```sh
exiftool -Sony2Fp:FocusPosition2 -FocusDistance2 shot_af.ARW shot_mf.ARW
```

- 两者皆为 `255` $\Rightarrow$ 镜头全程上报 `mag_code = 0xFF`；
- `AF` 为 `255`、`MF` 变为 `0..254` $\Rightarrow$ 仅 MF 分支计算倍率码；
- 全程在 `0..254` 随焦点单调变化 $\Rightarrow$ 具备完整原生 `mag_code` 上报。

---

## 4. Nikon Z 机身四域距离分离、五轴 `Property 0xA3` 与 `BVRCOM` 协同对照

在 Nikon Z8 (`v3.10`/`v3.12` `sec2_eg2070`) + IBIS 协处理器（`vr2070` RX 单元）与 [`z_snapshot_builder.c`](../../se-nz/src/02_core/z_snapshot_builder.c)、[`aperture_controls.c`](../../se-nz/src/02_core/aperture_controls.c)、[`optics_xlate_fpu.c`](../../se-nz/src/02_core/optics_xlate_fpu.c) 中，Z 卡口比 Sony 更彻底地把“物距/倍率”拆为四个彼此独立的目标槽位（均通过 `CMD 0x20 sub 0x00` `258 B` 状态帧下发）：

### 4.1 Z 机身的四个独立距离/倍率消费域

| Z 侧接口 / Property | `CMD 0x20` 帧偏移 | 编码与物理含义 | 机身唯一消费者 |
|---|---|---|---|
| **`Property 0xA3`** (`z_mag_a3_q14`) | **`TX[0xE3..0xE4]`** (`u16_le`) | **Q14 定点横向放大倍率**：$\mathrm{round}(16384 \times \beta)$；`1:1` 微距 = `16384`，`0` 或 `0xFFFF` 清零平移项 | **现代原生 `kind 4` 五轴防抖 X/Y 平移增益与像移目标**（`A3=0` 时仅关闭 2 轴平移，**3 轴 Pitch/Yaw/Roll 角度防抖 100% 保留**） |
| **`Property 0x117`** (`z_dist_0x117`) | **`TX[0x2F..0x30]`** (`u16_le`) | **16-bit 对数物距码**：$y = 20480 + 3072\log_2(D_{\mathrm{m}})$（`1/2/4 m = 0x5000/0x5C00/0x6800`，`0xFFFF` = 无穷远） | **NEF/JPG `LensData+0x4E..0x4F` EXIF `FocusDistance`**、对焦限位器 `a16`（仅老式 `legacy` 支路才会查距离表按 `F/(D-F)` 估算倍率） |
| **`Property 0xEA / 0xEB`** (`dnorm / fnorm`) | **`TX[0x31..0x34]`** (`2×u16_le`) | Q12 无量纲归一化对焦轴位（`0..4095`）与归一化焦段坐标（`0..4095`） | **机内 `CMD 0x24` 三维暗角/畸变/LCA 光学修正表双线性插值** |
| **EVF MF 标尺步进桶与门控** | **`TX[0xB9..0xBC]`** + **`TX[0x2D..0x2E]`** | `z_steps_0xbb ∈ [0, 255]`（`255` 阶感知曲线桶，同步写入 `CMD 0x74` 与 `LensData+0x58`）+ 查表 24 阶近距门控 `MF_GATE_TABLE` | **EVF/LCD 取景器 MF 对焦距离条与手动辅助显示** |

### 4.2 Sony SteadyShot 协同 vs Nikon `SyncVR / BVRCOM`（方向定律的对立）

当镜头自身也具备光学防抖（OSS / VR / VC）时，两大系统在**协同数据流方向**上完全相反：

| 维度 | Sony `A1 II`（BIONZ XR） | Nikon Z8 / Z9（`eg2070 + vr2070 BVR`） |
|---|---|---|
| **传感器上行规则** | **`GyroStart / GyroStop` + `gyro_sync_id`**：镜头原始陀螺数据流直接上行汇入机身融合环 | **方向定律**：**镜头陀螺原始数据永不回传机身**，镜头只在自己第二 MCU 内闭环消费自身陀螺 |
| **协同分工本质** | 数据融合级：主机 `smod` 融合机身陀螺 + 镜头陀螺 + Hall，`DividedInfoSteadyShotInfo` 对齐状态，按 `us_steadyshot_focal_length` 与仿射位移式协同 | **位移记账级**：机身 `VRCALC` 计算 5 轴总目标 `(dXopt, dYopt)`；镜头只通过窗口协议/块帧回报 2 轴**已施加光学位移 `(dXlvr, dYlvr)`**（物理上光学镜组不能转 Roll）；机身 IBIS 执行 **Roll + X/Y Shift + `(dXdiff, dYdiff)` 残差补偿** |
| **大转角溢出处理** | 融合环同步分配 | 镜头触及 VR 机械行程限位后，残差增量自动全部转落机身 5 轴 IBIS |
| **E→Z 转接桥接启示** | — | 由于 Z 机身不接镜头陀螺，任何把 E 口 OIS 镜头接到 Z 机身的转接环均必须采用 Nikon 记账式 `2+3` 分轴结构（即 `SE-NZ` [vr_duty_controller.c](../../se-nz/src/02_core/vr_duty_controller.c) 在 `CMD 0x12[0x39..0x3A] = 94 C4` + `CMD 0x20[0xE9..0xF0]` 下声明镜头承担 `2` 轴 Pitch/Yaw、机身承担 Roll + 近摄残余轴，并在 `D4 A3 00` / 关机下切 `0x34` 锁中停镜） |

### 4.3 `SE-NZ` 对 `Property 0xA3`、MF 标尺与 `255` 异常的工程处理（`V2.19–V2.23`）

1. **三级 `optics_compute_a3_q14()`（[`optics_xlate_fpu.c`](../../se-nz/src/02_core/optics_xlate_fpu.c)）**：
   - **Tier 1（原生倍率直通）**：当 `ID 0x05 grp_present[4] && mag_code != 0xFF && mag_code != 0` 时，直接解码 $\beta = 2^{5-\mathrm{code}/16}$，写入 `z_mag_a3_q14 = clamp(round(16384 * β), 0, 32768)`；
   - **Tier 2（镜头库 + `ID 0x06` 行程插值）**：对不报 `mag_code` 但已建档的镜头（如腾龙变焦群），按当前焦距插值 MFD 端点倍率并乘以 `ID 0x06` 归一化行程比例 `z_dnorm_0x31 / 4095`；
   - **Tier 3（几何物距兜底）**：按校验后的物理物距 $D$ 计算高斯近摄倍率 $\beta = \frac{f}{\max(f, D - 2f)}$（非微距头钳位保护上限 `4915`）。
2. **MF 标尺游标与 EXIF 距离分别计算（[`aperture_controls.c`](../../se-nz/src/02_core/aperture_controls.c) `focus_position_bucket()`）**：
   - `z_steps_0xbb`（`1..255` 感知步进桶）优先直接读取 `ID 0x06` 归一化马达行程 $t = \mathrm{cur\_cz} / \mathrm{mech\_hz}$，按 $0.75\sqrt{t} + 0.25t$ 平滑映射到 `1..255` 阶；只有当 `ID 0x06` 不可用时才退化查 `0x117` 距离码；
   - `z_dist_0x117` 则在连续对焦编码器算出的运动学距离 `kin_code` 与 `ID 0x05 dist_code_x`（`2048 + 48 * dist_x`）之间做滑动偏置管道融合（`fuse_cam_and_kin_dist_code`），兼顾连续高精度与真实标定原点。
3. **无黑名单行为式 `255` 拦截与自恢复（`KB-19` `aperture_controls_validate_distance()`）**：
   - 若某支副厂镜头在远景下将 `dist_code_x` 卡死在 `255 (0x00FF)`（`bad_reason = 2`）、输出 `0xFF00 / 0x7F00..0xFFFE`（`bad_reason = 1`），或跨过 `>= 220/1024` 对焦行程而 `dist_code_x` 纹丝不动（`bad_reason = 3`）：
   - 立即置 `dist_report_invalid != 0` 并记录黑匣子事件 `BB_EVT_DIST_FALLBACK (0x000E)`；同时在 [`z_snapshot_builder.c`](../../se-nz/src/02_core/z_snapshot_builder.c) 中撤下 `CMD 0x10[0x07] = 0`、`CMD 0x12[0x31..0x32] = 0x0000`、`CMD 0x20` 距离/MF 字段（驱动 Z8 返回无错乱的 `(-1, -1)`）并将 **`z_mag_a3_q14 = 0`**——防止远景下因假 `0.25 m` 物距灌入巨额平移前馈造成 IBIS 反向拉糊，完整保住 3 轴机身角度防抖、镜身 OIS 与 `ID 0x06` 闭环自动对焦；一旦镜头在 MF 下或更新固件后开始合法变化，`dist_verified_dynamic` 自动放行恢复显示。
   - **对照早期 `ETZ21 Pro Plus 0207`**：旧版 `0207` 将 `A3` 粗分为 `36` 桶步长（`A3 = 75 * k`），且在 Tamron/无符号高位 `40000..50000` 区间因坐标溢出将 `A3` 错误归零或保持旧帧残留倍率。

---

## 5. 双卡口自动对焦（AF）控制闭环与 Sony 原厂三代指令演进

无论是 Sony 还是 Nikon Z，**自动对焦主控制环都不读物距码或 `CamCurveData` 凸轮表，而是完全运行在“实时编码器轴位 + 像面离焦灵敏度斜率”闭环之上**。

### 5.1 Sony E 卡口 AF 闭环与原厂三代驱动指令族（`A1 II` / `A9 III` 实证）

- **核心反馈（`ID 0x06`）**：实时轴位 `focus_pos_se`、行程极限 `lim_far_se / lim_near_se`、运动标志 `hw_moving` 与每帧实时更新的 **`defocus_k_se`**。
  - 在 `A1 II` `0x00AA2A32` / `A9 III` `0x00AA4084` 中，机身直接用镜头报出的 `defocus_k_se`（标准格式在镜头内部已预乘 $0.012\,\mathrm{mm}\cdot F_{\mathrm{cur}}$，扩展 `tag 0x44` 则直接报纯几何 $2\,\mathrm{d}S_E/\mathrm{d}z_{\mathrm{mm}}$）算出目标编码器脉冲步长 $\Delta S_E$——**放大倍率衰减 $(1+\beta)^{-2}$ 与内对焦呼吸斜率均由镜头在 `defocus_k_se` 内预先消化完成**，这正是副厂头即便把距离和倍率都填 `255` 仍能正常 AF 的根本原因。
- **三代驱镜指令（嵌在每帧 `ID 0x04` 内，受 `CMD 0x01 / 0x5A` 能力位图严格门控）**：
  1. **第一代通用基础层（全副厂及早期头通用）**：
     - **`0x1D`（`5 B` `"BL:MOVE_FC"`，`A1 II 0x00432A80`）**：发送绝对目标位（`b4 = 0x00` 停稳 / `0x80` 连续追焦）或 `s16/s24` 相对步进（`b4 = 0x2C/0x2E`，`family=2` 时 `b4 = 0x3C`）；
     - **`0x3C`（`8 B`）**：受控速度恒速扫描/视频推进；
     - **`0x1C`（`1 B`）**：立即制动停稳；**`0x1F`（`14 B`）**：附加廓线移动指令。
  2. **第二代原厂双/多 XD 扩展层（`0x41 / 0x43 / 0x44 / 0x49` 能力簇）**：
     - **`0x43`（`12 B` `"BL:MOVE_FC_A"`，`A1 II 0x00432B04` / `A9 III 0x00435E90`）**：载荷为 `(seq, abs_se, rel_shift, wob, ctrl_flags, cnt, fc_start_time)`，携带同步计数与驱动启动时间窗，并搭配 `0x49` 辅助驱动通道，用于原厂多 XD 线性马达与浮动镜组与快门时序的精确合拍。
  3. **第三代高速预测轨迹层（`A9 III` 新增）**：
     - **`0x6B`（`28 B` `"BL:MOVE_FC_TRAJECTORY_A"`，`A9 III 0x00435FA4`）**：携带 **5 点位置轨迹数组**，用于 `120 fps` 全域快门每 `8.3 ms` 一帧的连续运动预测追焦——让镜头马达按连续轨迹滑行而不必每帧停顿等待下一拍指令，这也是原厂头解锁 `30 / 120 fps` 满速追焦而副厂受限于 `15 fps` 的底层协议分水岭。
- **配套高压与高刷握手命令**：
  - `CMD 0x4C` $\to$ `CMD 0x4B 60 60`：协商解锁高功率马达供电轨，释放 XD 线性马达峰值推力；
  - `CMD 0x5F`（`"LENS_DRIVE_FREQUENCY_A"`）：在 `CMD 0x0C` 协商至 `1.5 Mbps` 后，把驱动/采样频段从 `60 Hz` 切换至 `120 Hz` / `240 Hz`；
  - `CMD 0x3D`：读取双对焦组标志（`0x42`）及超声基频参数；
  - `CMD 0x32` + `CMD 0x5A`（`56 B`）：双向能力掩码裁剪——老机身或转接环只要在 `CMD 0x5A` 中清除 `0x43 / 0x49 / 0x6B` 位、保留 `0x1D` 位（`SE-NZ` 即按此路线），连 `FE 50–150mm F2 GM` 这类最新 `family=2` 旗舰头也会完全平滑地退回 `0x1D / 0x3C` 闭环正常高速工作（并经 `A1 II` 单比特关 `cap(0x43)` 机器执行复证）。

### 5.2 Nikon Z 卡口双总线 AF 闭环与 `SE-NZ` 防双除耦合

1. **双 SPI 分离时序**：
   - **`SPI1` `1 kHz` 热线（[`z_spi1_hotline.c`](../../se-nz/src/01_transport/z_spi1_hotline.c)）**：每 `1.000 ms` 连续发 `42 B` 状态帧，核心为 `s32_le C_Z`（双曲倒数米制位置 $C_Z = \mathrm{round}(A / D_{\mathrm{m}})$，`A = 2^{n+7}` 由 `CMD 0x10[0x0B]` 选档）、端止状态 `HOT[7]` 与运动停稳位 `HOT[6]`。Z8 `0x40DCC854` 要求热线连续 **`>= 5` 帧**报静稳（经 `0x41078348` 投递 `event 0x3C = 2`）才将对焦框转为绿色合焦。
   - **`SPI3` 异步控制命令（[`af_servo_bridge.c`](../../se-nz/src/02_core/af_servo_bridge.c)）**：随用随发 **`CMD 0xC0`**（相对脉冲 + 有符号 7-bit 速度字 + 绝对护栏 target + `mosi[11]` 标志）、**`CMD 0xE4`**（AF-C 双目标轨迹预测）、**`CMD 0xE2 / 0xE6`**（步进微调与绝对跳转，注意其刹车判断是 `(mosi[7] & 0x06) == 0x06` 而非 `C0/E4` 的 `!= 0`）、**`CMD 0xE8`**（反差搜峰微摆）及 **`CMD 0xE0 / 0xE1`**（急停/软停）。
2. **`K_Z`、出瞳加权与单次 $(1+\beta)^{-2}$ 律**：
   - Z8 使用 `CMD 0x20[0x6D..0x72]` 的双迷你浮点 `kz_b0 / kz_b1` 计算步数，并在进入搜峰前通过 `0x40DCA360` 检查 `[0x15..0x24]` 出瞳光线几何门控；
   - 转接环发布 `CMD 0x12[0x1A..0x1B] = 0x0100`（`KZ_NONLIN_DISABLED`）禁用机内残余非线性查表，改由转接环按连续雅可比 $J = |\partial C_Z / \partial S_E|$ 与光圈去耦+单次 $(1+\beta)^{-2}$ 上限包络（`af_servo_refresh_kz()`）实时喂给 Z8 精确线性 `K_Z`。

---

## 6. Sony E × Nikon Z × SE-NZ 全功能字段消费速查总表

| 机身功能 | Sony E 原厂机身消费字段 | Nikon Z 原厂机身消费字段 | `SE-NZ` 桥接固件核心行为（`V2.23`） | 镜头把距离/倍率全填 `255` 时表现 |
|---|---|---|---|---|
| **自动对焦位置反馈** | `ID 0x06 focus_pos_se` / `lim_far..lim_near` | `SPI1` `1 kHz` 热线 `HOT[2..5]` (`C_Z`) + `HOT[6..7]` | `ID 0x06` 无符号安全解包 + 变焦齐焦零点锚定外推 `1 ms` `C_Z` 热线 | **完全不受影响** |
| **离焦脉冲换算灵敏度** | `ID 0x06 defocus_k_se`（或扩展 `tag 0x44`） | `CMD 0x20[0x6D..0x72]` `kz_b0/kz_b1` + `c0/c1` + `[0x15..0x24]` 出瞳门 | 消除 Sony $F_{\mathrm{cur}}$ 因子 + 单次 $(1+\beta)^{-2}$ 包络 + 雅可比 $J$ 换算 + 禁非线性表 | 仅以 $\beta = 0$ 回退，AF 主环仍闭环工作 |
| **AF 马达驱动指令** | `ID 0x04`：`0x1D` / `0x3C` / `0x1C`；原厂扩展 `0x43` / `0x49` / `A9 III` `0x6B` | `SPI3`：`CMD 0xC0 / E2 / E4 / E6 / E8 / E0 / E1` | `CMD 0x5A` 掩码保留 `0x1D`；Z 控制字双极性解码路由至 `0x1D / 0x3C / 0x1C` | **完全不受影响** |
| **EVF MF 距离标尺游标** | `ID 0x06` 归一化行程 + `ID 0x05 grp[7]` 环信号 | `CMD 0x20[0xBB..0xBC]` (`z_steps_0xbb`) + `CMD 0x74` + `[0x2D..0x2E]` | `focus_position_bucket()` 优先按 `ID 0x06 cur_cz/mech_hz` 计算 `1..255` 桶 | **游标可正常移动**（触发 `KB-19` 异常拦截时隐去以防错乱） |
| **EVF 数字米制 / 限位器** | `ID 0x05 grp[4]+0..1` `dist_code_x`（部分UI显示） | `CMD 0x20[0x2F..0x30]` (`Property 0x117`) | 连续运动学距离与 `dist_code_x` 滑动融合；Z 限位器按 $A/D_{\mathrm{lim}}$ 毫米对齐 | Sony 下可无显示或仅 MF 有值；`SE-NZ KB-19` 检出卡死后主动隐去假距离 |
| **五轴防抖：3 轴角度** | 当前焦距 `us_steadyshot_focal_length` | `CMD 0x20[0x0D..0x0E]` / `[0xE1..0xE2]` `f_eff` | 按 `ID 0x05 grp[4]` 标称/呼吸焦距及 `DS13` 实时发布 | **100% 保留工作** |
| **五轴防抖：2 轴平移（近摄）** | **`ID 0x06 focus_pos_se` + `CMD 0x3A/3B CamCurveData (fc_pitch)` + `SteadyShotInfo`** | **`CMD 0x20[0xE3..0xE4]` (`Property 0xA3` Q14 $\beta$)** | 三级优先级生成 `z_mag_a3_q14`；`KB-19` 检出冻结/卡死 `255` 后强置 `0` | Sony 看有无 `CamCurveData`；`SE-NZ` 主动置 `A3 = 0` 防止假近摄平移糊片 |
| **机身 ↔ 镜头双防协同** | `GyroStart` 镜头陀螺上行进机身融合环 + `SteadyShotInfo` | `BVRCOM` 记账式：镜头只报 2 轴 `dXlvr/dYlvr`，机身补 5 轴残差 | `vr_duty_controller` 以 `94 C4` + `[0xE9..0xF0]` 实现 Nikon `2+3` 分轴与关机锁中 | 无防抖头走 `00 44` 纯机身 5 轴；有 OIS 头按状态分轴 |
| **机内畸变/暗角/LCA 轴坐标** | 镜头光学曲线组 `ID 0x05 grp[5,11..13,15..17]` / `DS0` | `CMD 0x20[0x31..0x34]` (`Property 0xEA/0xEB` Q12) + `CMD 0x24` | 由 `ID 0x06` 与焦距生成连续 Q12 `(dnorm, fnorm)` 插值表 | `dnorm` 来自 `ID 0x06`，**基本不受影响** |
| **照片 EXIF 对焦距离** | `ID 0x05 grp[4]+3` `mag_code` $\to$ `Sony2Fp.FocusPosition2` | `CMD 0x20[0x2F..0x30]` (`Property 0x117`) $\to$ `LensData+0x4E..0x4F` | 由实时物距链路填充 `z_dist_0x117`；触发 `KB-19` 时置 `0` 清空无效 EXIF | Sony ARW 恒记 **`255`**；`SE-NZ` 撤下无效物距防止误记 `0.25 m` |
