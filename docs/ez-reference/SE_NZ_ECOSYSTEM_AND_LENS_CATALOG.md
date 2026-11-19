# SE-NZ 工程生态与镜头目录总规格书（V2.23 基线）

目标平台：`HC32F460JEUA`（ARM Cortex-M4F @ `192 MHz`），Sony E 卡口镜头 → Nikon Z 卡口机身（源码目录：[`se-nz/`](../../se-nz/)）。
本文汇总硬件接口合同、双侧线协议、会话恢复时序、对焦/光圈/防抖/光学控制律、Nikon Z 机身矩阵、跨平台转接环对照，以及由 [`lens_db_generated.h`](../../se-nz/include/lens_db_generated.h) 与 V2.23 构建产物自动生成的 `275` 项镜头目录与 `57` 条光谱映射附录。

## 1. 系统目标与证据口径

固件负责镜头枚举、机身身份与能力申报、AF/DMF 指令桥接、光圈、实时焦距、距离与 EXIF、OIS/IBIS 分工、光学校正表、关机停车及恢复。
主体识别、目标选择、相位测距和拍摄策略由机身完成；镜内电机电流环、机械限位与校准由镜头控制器完成。

本规格书按以下证据落表：

| 标记 | 含义 | 可复核入口 |
|---|---|---|
| CAT | 固件目录身份/参数条目 | `include/lens_db_generated.h` |
| ROM | 原始镜头或机身固件提取 | 下文资料索引的二进制、CSV、JSON |
| NATIVE | 对原始机器指令执行的消费端验证 | Z8 AArch64、E6 Thumb 消费端 |
| CORE | 共享 C 核心的 ASan+UBSan 线协议检查 | M1 / protocol matrix |
| MCU | APP+原厂 BL 的指令级外设仿真 | M0 / hardware stress |
| DEVICE | 指定包、实体机身、实体镜头的观测 | 实机黑匣子与闭环遥测记录 |

目录条目、跨卡口光学数据移植、模型测试和实体设备记录各自使用上述标记。软件验收按实际用例计数；实体组合的 AF 精度、马达声响和摄影结果由 DEVICE 样本闭环。

### 1.1 AF 验收覆盖核查

当前 AF 证据具体覆盖：

- M0 的 Sony 复合检查以 `focus_se != 2000` 判断发生过运动，并核对驱动帧和运动相关状态。
- M1 的 AF-ON 完成场景由测试注入目标 500 Z 对应的 `pos_se=2146, moving=0` 反馈，检查相应状态处理。
- 协议压力的大量断言检查帧校验、目录、命令回显与状态切换。

AF 功能放行增加独立行为门槛：静态目标位置误差随驱动反馈收敛、停止后位置保持、连续移动目标误差、运动中反向重捕获、AF-ON 释放/重按、DMF 插入与退出。每个场景记录目标、实际反馈、驱动输出、收敛时间及终态，分别标明模型参数与实体观测来源。既有协议用例数量按其原检查对象统计；AF 行为验收单独计数。

新增 `test_af_feedback_behavior.py` 将实际位置交由独立有限速度执行器更新，驱动帧只设置目标；ID06 由执行器状态产生。输入为 60Hz、600 encoder/frame、1/3 帧命令延迟、0/2 帧反馈延迟，覆盖静态目标双向重捕获、三角轨迹、AF-ON 停止重触发与 MF 插入退出。静态判据为末 8 帧误差≤16 Z 单位、执行器停止且 HOT Busy 清除；轨迹判据为启动后误差≤1600 Z 单位并在停止段收敛。完整逐帧误差保存在 `evidence/af_feedback_behavior.json`。停转、反向及绝对落点欠走 100 encoder 的负例均由同一收敛判据拒绝；反向和欠走用例另确认执行器确实发生运动。

这些执行器参数是显式敏感性模型输入。超远摄与大光圈双浮动代表样本（如 **Sigma 150–600mm F5–6.3 DG DN OS Sports** 与 **Sony FE 50–150mm F2 GM**）：实体 AF 质量沿 Z8＋该镜头的 150/300/600mm 同步目标、画面焦差与 E/Z 反馈闭环，标定响应延迟、速度和焦差容差。Sony 50–150 同步按能力协商与响应模型推进。

## 2. 架构与执行顺序

```text
原厂 BL / UID 配对
  → Reset_Handler：SRAM 初始化、ramfunc 搬运、VTOR、FPU
  → GPIO 冷启动 → 时钟/电源 → TMRA1 → Z SPI3/SPI1 → 会话与日志
  → 主循环
      Z 窗口 / Hotline
      E USART DMA / 握手与遥测
      VSync ID03 / ID04
      热插拔 / 停车 / 唤醒
```

| 模块 | 文件 | 责任 |
|---|---|---|
| 启动/寄存器 | `src/00_hal/`、`include/hc32f460_regs.h` | 时钟、GPIO、单调计时、EFM |
| Z 主通道 | `src/01_transport/z_spi3_slave.c` | header/payload 双窗口、DMA、RDY |
| Z Hotline | `src/01_transport/z_spi1_hotline.c` | 42B 周期快照、序号与槽位 |
| E 传输 | `e_usart1_dma.c`、`e_tmra4_vsync.c` | 物理握手、变速、接收分帧、同步相位 |
| 会话 | `src/02_core/lens_session.c` | 能力交集、身份、DS13、组掩码、状态机 |
| AF | `af_servo_bridge.c` | 坐标映射、限位、速度、预停、DMF |
| 光圈/距离 | `aperture_controls.c` | 光圈码、MF 标尺、EXIF、L-Fn |
| 防抖/停车 | `vr_duty_controller.c` | 轴职责、关机停流与锁定 |
| 光学/快照 | `optics_xlate_fpu.c`、`z_snapshot_builder.c` | 大表、双缓冲、响应编码 |
| 记录 | `src/03_diag_storage/` | blackbox、原子 journal、复位痕迹 |

主循环执行浮点换算；硬件中断、EFM 原语及打包器指定的实时函数走整数路径。`.ramfunc` 的 SRAM 位置、传递调用与浮点指令由打包门禁和独立审计共同检查。

Z 命令轮询与分派位于主循环 Flash 路径，可调用浮点业务核心；SRAM 中的 SPI DMA 装填/重臂原语保持整数闭包。打包门禁同时执行 HAL/Transport 本体扫描、SRAM/ISR 传递闭包扫描，以及 EFM 闭包的 SRAM 驻留检查。

## 3. 硬件合同

### 3.1 引脚与物理角色

| HC32 引脚 | 角色 | 外设/行为 |
|---|---|---|
| PA0 | E7 镜头 → 接环 RX | USART1 RX |
| PA1 | E8 接环 → 镜头 TX | 初始低；握手后 USART1 TX；切速锁高 |
| PA2 | E9 接环 CS | 发送包围、唤醒握手 |
| PA3 | E10 镜头检测 | 上拉输入；物理低表示在位，API 返回 1 |
| PH2 | E6 镜头 CS | 高起包、下降沿结算 DMA 接收 |
| PC13 | E4 VSync | TMRA4 PWM8，60/120 Hz |
| PB13/PB14/PB15 | Z9 MISO / Z8 SCK / Z7 MOSI | SPI3 从机 |
| PA9 | Z6 RDY | 高 Busy、低 Ready |
| PA11/PA12 | Z10 HCLK / Z11 HDATA | SPI1 Hotline |
| PH0/PH1 | 外部晶振 | 16 MHz → MPLL |
| PA13/PA14 | SWD | 保持调试用途 |

冷启动预锁存 PA9=0；首次开窗经过 SCK 连续高 5 ms，再以 Busy 脉冲装填 SPI3/DMA，Busy 至少 500 μs 后 Ready。运行中重置或释放 Z pads 时 PA9=1。

### 3.2 时钟、DMA 与中断

时钟：SYSCLK/HCLK 192 MHz，PCLK1 96 MHz，PCLK2/3 48 MHz；TMRA1 3 MHz。
升频顺序：SRAM 等待 → EFM 等待与缓存 → UHS 电源 → 外晶与 MPLL → 主时钟切换。精确寄存器以 `sys_clock_sram.c` 为执行依据。

| 功能 | 通道 | 事件/中断 |
|---|---|---|
| Z SPI3 RX / TX | DMA1 CH0 / CH1 | 主循环检查两路 TC |
| E USART1 RX | DMA2 CH2 | PH2 下降沿结算；4×2048B 槽 |
| E USART1 TX | DMA2 CH3 | IRQ41 / event 39 |
| Z SPI1 TX | DMA2 CH0 | IRQ40 / event 36 |
| E4 VSync | TMRA4 | IRQ80 / event 265 |
| 单调时基 | TMRA1 | IRQ82 / event 256 |

USART 使用 8N1，750 kbit/s 的 BRR=`0x0FFF`，1.5 Mbit/s 的 BRR=`0x07FF`。发送首字节由 PIO 写 TDR 点火，其余 `N−1` 字节走 DMA。PA2 高沿后的定时基准直接读硬件计时器。

### 3.3 Flash 与 SRAM

| Flash 范围（左闭右开） | 用途 |
|---|---|
| `00000000..00010000` | 原厂 BL；含 `00004200` UID AES 块 |
| `00010000..00060000` | APP；向量表 `00010000`，版本槽 `00011000` |
| `00060000..0007A000` | 文件系统隔离/保留 |
| `0007A000..0007C000` | blackbox，256×32B |
| `0007C000..0007E000` | journal A |
| `0007E000..00080000` | journal B |

`SCB->VTOR = 0x00010000`。启动向量的 MSP=`0x1FFFFF00`，主栈底=`0x1FFFE000`，顶部保留 256B。
SRAMH `1FFF8000..20000000`；SRAM1 `20000000..20010000`；SRAM2 `20010000..20020000`；SRAM3 `20020000..20027000`；RetRAM `200F0000..200F1000`。
SRAMH 保存 `.ramfunc`、DMA 和栈；SRAM2 保存双缓冲光学校正、3872B DS13 与 36KiB LF 工作区。实际 ELF 段占用见生成附录。

## 4. 两侧线协议

### 4.1 Z 主通道

SPI3：Mode 3、LSB-first、16-bit 硬件移位、小端内存直通。Header 8B：
`55 00 00 cmd sub len_lo len_hi checksum`，payload 字节数=`2×(len16+1)`，负和校验使整段和模 256 为零。
先完成 header DMA，再装填 payload DMA，处理 MOSI 后恢复 header。
大于 512B 的 MOSI 接收走固定 trash sink；TX 大表使用已有快照或大缓冲。

| Z 命令 | 主要响应/用途 |
|---|---|
| 10 / 11 / 12 | 28B 身份、74B 名称、66B 能力与光学参数 |
| 15 | 握手 Busy=3，激活后 3→2→0 |
| 16 | Hotline 开启 |
| 20/00，20/01 | 258B 状态、24B 补充焦距 |
| 24 | 7426B / 3468B 光学校正快照 |
| 50 / 51 | 停车请求 / Busy 81、Ready 01 |
| 74/00 | MF 功能载荷 `55 01 23 00 03 01 00 83` |
| A0/A1/A3 | 光圈控制与参数 |
| C0..C4、E0..EE | 对焦命令族 |
| D2..D8 | 防抖控制族 |

不同机身描述符的 TX/RX 长度以 ROM 提取矩阵为准；非对称窗口按原码消费者要求形成响应。Phase-2 等待超过 1 s 且 SCK 高时恢复 header。

### 4.2 E 信封与组遥测

`F0 | u16_le length | type | seq | cmd | payload | u16_le sum16 | 55`。
总长=`payload+9`，sum16 累加从长度低字节至 payload 末尾。载荷内的 F0/55 按原始字节传输。
ID05 组长度：
`8,1,8,3,10,28,2,6,1,2,8,6,6,6,1,6,3,3`；前 15 组 96B，全 18 组 108B。
当前实现的关键组：grp0 光圈 P；grp2 出瞳；grp3 光圈环；grp4 距离/倍率/光学及名义焦距；grp5 像差；grp7 MF/L-Fn/OIS 开关。ID06 提供焦点编码器、端点、变焦坐标、灵敏度与运动状态。

### 4.3 能力交集与 Sony Gen-2

CMD01 机身能力 32B：`7×FF, 7F, 23×00, 80`。
典型扩展握手：`01 → 07 → 0C → 3F → 3D → 3A/3B(DS13) → 32 → 5A → 08 → 09 → 0B → 0D → 10 → 0A`，各步骤受能力位门控。
CMD5A 发送时掩码同步写回 `cap_bitmap`，使后续运行遵循协商能力。
`hs_cmd07_ok` 表示本次握手成功，旧镜头 ID 用于唤醒身份显示。

`has_ext_5a` 镜头使用 27B ID04/1D：相对步进后发绝对落点。空闲 ID04=22B。
普通 scan 镜头使用 30B/3C；运动中反向先发 23B/1C，停止后恢复相应驱动。
Samyang 家族按 TX07 的动态位选择 29B/32B ID03；目录强制项随后合并。
未知镜头从 CMD3F 可打印 ASCII 名称提取，最长 63 字符。

## 5. 状态、时序与失效恢复

状态集合：DETACHED、POWERUP_WAIT、HANDSHAKE、ACTIVE、PARKING、PARKED、EMPTY_MOUNT、PASSIVE_MOUNT。

| 边界 | 设计条件 |
|---|---|
| 冷启动慢镜头 | 110 ms 首窗守门；保留机身供电窗口 |
| 空环 | 固定 50mm/F1.4 手动身份 |
| 电子镜头热插拔/身份重枚举 | PA9 连续 Busy 至少 450 ms，再按 SCK 条件开窗 |
| CMD01 | 最多18次实际重试耗尽后进入静默回退；保留慢供电握手窗口 |
| 身份与长归位 | 6.5s总握手预算；CMD07合法非零身份是主动会话前提 |
| DS13 | 空会话与新 DS13 开始时清 len/valid |
| 唤醒 | 当前握手完成前 cmd15=3；重建 ID05/06 组掩码 |
| 停车 | 停流、CMD16、锁定等待；SCK 低时维持 SPI3 停止与 Busy |

重复停车请求保留进行中的停车状态与起始时刻。握手/归位期间的停车请求置 `park_after_handshake`，完成握手后执行完整停流及锁定序列；等待期间 cmd51 保持 Busy。
CMD07的迟到回复保持原接收阶段；短帧和零ID维持握手，阶段成功由当前ACK确认。450ms POR、120/450ms晚身份的实际MCU因果回归通过；电池电压及机身撤电源继续沿实体条件绑定。

周期调度：

| 频率 | ID03 最早/最迟 | ID04 最早/最迟 | ID03→04 间隔 |
|---|---|---|---|
| 60Hz | 7.50/13.50 ms | 9.00/15.50 ms | ≥1.00 ms |
| 120Hz | 4.00/6.40 ms | 5.00/7.60 ms | ≥0.80 ms |

120Hz 生效条件由运行期能力及品牌门控共同决定；当前 `finalize_active_session()` 为具备高速能力、已切 1.5M 的 Sigma 开启 120Hz，其余保持 60Hz。

## 6. 控制律与状态映射

### 6.1 焦点坐标与灵敏度

按最长焦距选择 `A = 16384 / 65536 / 131072`。
由近摄距离 `Dmin`、机械端点 `Einf/Enear` 得：

```text
Hz = round(A / Dmin)
J = Hz / abs(Enear − Einf)
Cz = clamp(round(sign(Enear − Einf) × (E − Einf) × J), -512, Hz)
Etarget = Einf + sign × round(clamp(Ctarget,-512,Hz) / J)
```

目标再按当前镜筒限位钳位。Sigma 150–600 使用13节点变焦凸轮；C551采用固定光学4096..28296，C572/C583以原光学锚点绑定full报告校准增量；其他镜头通过ID06扩展观测端点。
光学无穷远为0，负512为其外侧的虚拟余量；实际机械限位与光学锚点分别处理。
速度与 Kz 使用同一 Jacobian；近摄倍率、温度曲线和双浮动组降速进入限速计算。
机械编码器到物距的线性归一化依赖端点和目录 MFD；目录缺省 MFD 是工程估计，附录显式标出其来源。ID05 自带的距离码优先进入 EXIF。

K量纲采用原ETZ生产链：由E光圈Av生成`Fnum_Q8`，`divisor=Fnum_Q8*1792>>8`，`norm=floor(((K<<16)/divisor)*0.9)`，物理K=`norm/64`；再乘J及`1/(1+β)²`。原K为0时回退5.015625，归一结果量化为0时回退0.15673828125。Z8的K解码结果为Q16，打包前尺度为`×131072`。原3.10/3.12解码器23项对拍一致；旧标量1仅解码为128，修后65536。

### 6.2 AF、MF、DMF、AF-ON

驱动期间 ID04[9]=83；结束后约 150 ms 保持，随后 81。保持窗按 60/120Hz 换算为 9/18 帧。
AF arm 清除旧 MF 环遥测；半按 S1 的存在与 AF drive 分开判断。
绝对命令送出后，初始 idle 反馈继续保持驱动等待；只有剩余位置误差进入容差且反馈稳定停止时才确认完成。迟启动慢速场景覆盖 1 encoder/frame、3 帧命令延迟与 2 帧反馈延迟。
停滞计数按编码器位置实际变化复位；同一次活跃驱动中的连续目标更新保留进度计数。步进和绝对落点等待路径均在停滞超时发出 1C 停止。持续 Busy、位置冻结的故障检查将“按时中止”与“到达目标”分开判定。
AF真实到位后HOT[6] bits5:4回0成功；运动等待为1、错误为2/3。`arrival_confirmed`由目标残差及停止/稳定反馈建立，新目标或显式停止清除。原Z8保存位置完成消费者的两镜成功事件与欠走等待负例分别执行。
MF 标尺链：cmd12[27..28]=03 FE → 机身 MF 选择 → 全光学行程0..35档 → 索引`floor(steps*2/3)`的24项gate → cmd74。有效ID06反馈使用位置分档；距离/EXIF保持独立原距离码。
HOT[21] 优先使用有效 ID06 环方向，保持窗内以 ID05 环增量方向补齐。
HOT[6].bit7按原V120兼容策略镜像CMD20[2]的所选/有效MF标志；原Z8按焦点上下文发布并维护状态缓存。源码及对应回归采用MF命名，半按/曝光由独立拍摄事件合同表达。

### 6.3 光圈与 EXIF

`Cz_aperture = 384 + 192×2log2(F)`；
`P_e = floor((4×Cz_aperture+1)/3) + 0x0E00`。
回读按 P 的整数/小数部分换算并钳位。光圈环手动状态与机身目标按当前环模式选择。
距离码 `Z117 = clamp(2048+48×E_dist_x, 8, 65527)`；无穷远输入65535对应输出65535。MF标尺采用上一节有效位置分档；缺少grp4时，距离采用编码器估算的`20480+3072log2(Dm)`。

### 6.4 防抖、光谱与校正

`00/44` 与 `94/C4` 分别承载不同 OIS/IBIS 能力申报。D2 Normal/Sport、D4 Off、镜筒拨杆共同决定镜头和机身轴职责。具体因果输出由 Z8 原码轴权重消费者核验。
Normal菜单发D2/01，Sport菜单发D2/04。活跃photo阶段W19=03；B21由使能、Normal门控bit1/Sport持续策略、镜筒独立模式bits3:2组成。原Sony/A067开关字段零基0/1/2，在有效带开关镜头转换为B21模式1/2/3；A065按无模式开关处理。旧Sport09及OS2的07已由原photo生产合同修正，09实际属于movie-policy路径。
Sigma OS1/OS2与Custom OFF/C1/C2保持独立，OS1×三档Custom均可用于体育；十二组合原RX→副控回归169项通过。D2无条件1901已移除，原曝光覆盖位在取景保持0。CMD20[2].bits1/2为MF字段，生产已与VR曝光位解耦；真正S1、曝光进入/退出及ID19有序配对按Z端真实事件流接续。
C4同时参与模式发送资格与释放等待。D2/D4建立模式状态；D5/03锁存停止，wire DF回00，保持能力、F2及轴数据；持续下发E ID03 `00/0033/00`，直到新模式请求。C56E原接收→策略→SPI输出S，再由镜头内部ADC窗口与phase决定居中完成。该版本ID05的两位OIS反馈为保留0位；Z端停止回报与镜内物理完成采用各自判据。完整原UI→capture→D5→CMD20→许可链覆盖初始pending0与模式后pending1。

Sony807B按官方无OSS规格申报00/44，由机身负责五轴防抖；实际VD60配0D00。原Sony基准档60/50/48分别编码00/01/02。
光谱载荷为 179B，包含 81 点 380..780nm、步长 5nm 的主曲线。当前池含 58 条：13 条 Tamron Z 来源、44 条 Sigma 来源与一条 Samyang 35–150 来源。
Sigma 41 点输入按生成器定义的 350..750nm 网格重采样，末端使用线性外推并钳位；跨 E/L 或 E/Z 使用同光学设计曲线属于数据移植。
其余目录镜头使用中性回退。Sony DS13 用于呼吸/有效光学量修正，独立于光谱池。
传感器消费者的核心积分为 `M[c,j]=Σ Lsensor[i,c]×Tlens[i]×Rilluminant[i,j]`，再用参考镜头与光源权重算 Q15 色增益。

原生 Z 参考属性 `(1,0)` 选择 ROM_B；旧夹具 `(3,4)` 选择 ROM_A。两组参考分别保留因果对照，M1 与四品牌链采用原生 Z 参考。150–600 新光谱曲线对应的原码 Q15 结果为 `[31828,34449]`。

### 6.5 原生校正模板与 DN 转译合同

Tamron 13 个同设计 Z profile 保留完整 A（7426B）与 B（3468B）模板；A 的 1620 浮点主张量、54 项畸变以及 B 的 864 浮点张量随原始节点和标志保存。实时 ID05 保持该原生模板；光谱段与发送校验按协议更新。逐 ROM 地址、完整 B 字节匹配和 A084 原码发送/坏校验反例见 `brand_source_followup.md`。

DN 原码消费者的正确数学合同为：

| 数据 | 原码合同 | 实现接入条件 |
|---|---|---|
| 54 项畸变 | `3×6×3`，状态 `[1,sqrt(f),f,f²,f³,f⁴]×[1,d,d²]`，径向 `a·r⁴+b·r³+c·r²` | 当前状态常量槽 `0/18/36`，百分比封装 |
| A 增益 | `t,t²,t³(,t⁴)`，257 点 Q10 | 光圈节点轴与焦距/距离状态轴分别拟合 |
| B 红/蓝色差 | 各 `1,t²,t³`，各 17 个有符号 LUT 值 | 绑定源单位、半径与裁切 |
| 表发布 | parser/ring 接收后机身驻留 | 全状态张量或经过验证的重取表时序 |

早期光学消费端对照记录 保留早期150/300/600mm×远近两状态、30条原始曲线及699/699算术断言。原 `ID05` 帧实测对拍进一步确认：真正畸变为selector21，红/蓝色差为22/23；旧SetA/B为selector3/4独立记录，早期结果按该角色修正理解。

后续 ISP 像素尺度与裁切合同记录 已闭合倍率115/128、Q4位移、H=416、参考半径5199、声明全幅图像半径5192、裁切21494/32768及逐状态偏置，Set B 当前状态拟合残差降至0.599–3.894px。原C25/C26完成路径复制驻留表并切换银行，生产接入采用完整多状态张量。

当前A表已接入真selector21驻留54项模型及flag3完整暗角张量。多状态畸变/暗角张量拟合记录 使用207状态、45拟合/162保留验证；1679/1679检查通过。分离Sony中心倍率后，真畸变独立径向最大残差0.378084px、完整张量保留状态3.271852px。暗角3933组原消费者257点算术一致，有效图像域最大相对残差6.522%。原selector3/4候选及旧边缘保持拟合留在独立历史文件；B表下一步接真22/23的ISP像素单位合同。

### 6.6 代际反馈与扩展响应

- C56E 原生 TX07 为 family1/protocol0190；TX06 的 39/88B 载荷中 `[2..3]` 为发布位置、`[20..21]` 为实际采样。消费端按该代际取后者，回归刻意分离两字段。
- Sigma AF 运动的 `02/04` 与 payload[6].bit4 联合解释；标志有效时，直接状态与编码器位移推断均保持 AF 身份。标志清除后的真实手动环另有对照。
- `1F` 的参数 `08` 写入独立参数字段；原执行状态由消息入口置 1、3C 入口置 6。初始化完成后按能力发送扩展请求。
- Sigma 3E 为 20B 线包/11B payload：`0..1` 环 Av、`2.bit0` 手动标志、`3..4` 叶片 Av。C583 原码 A/手动 F4 响应与其他型号合法光圈输入共同覆盖查询、仲裁与回写。
- Tamron 位置和端点按无符号 16 位解释；32767→32768→40000 保持连续，绝对 1D 输出保留 40000。
- 普通镜头以首次有效 ID06 端点建立会话坐标并缓存、扩展；实时 limiter 独立钳位。150–600 DN 保留专属变焦凸轮。

### 6.7 LCA 像素合同与轮询缓存

LCA Bayer Q6 像素合同分析 的718项原指令检查经主线程复跑通过。Nikon软件像素消费者以Q6表示原始Bayer像素位移，正值向更大输入坐标取样；`ShiftFluct=3`时signed word对应Q16径向倍率。非线性17节点均匀分布于半径平方。SDK使用10bit钳位，Z8真实writer使用signed11编码；目标模式的ShiftFluct与半径参数按原机身生产者绑定。

ID05在两个槽中轮询selector21/22/23。生产解码器现将各类型的完整9B记录分别缓存，并分别记录核心有效位和完整尾段位；截短帧保持已有缓存，6B输入清除该类型旧尾段，其他selector保持对应独立记录。`test_id05_selectors.py` 以6状态、12份原C56E ID05帧覆盖换槽、旋转、截短、未知selector和尾段缺失，36/36通过。

最终B表物理转换接续两项原接口：Sony有效ISP／ARW7035同值链，以及Z8当前模式ShiftFluct、半径乘数/右移和光心。对应原码生产者集中记录在 LCA 与 Z8 ShiftFluct 终态绑定记录。

## 7. Nikon Z 机身矩阵

官方目录在 2026-11-14 抓取：15 款 Z 无反加 ZR，共 16 个 Nikon 条目。
入口：[Z 无反](https://imaging.nikon.com/imaging/lineup/mirrorless/)、[Z CINEMA](https://imaging.nikon.com/imaging/lineup/z_cinema/)。

| 机身 | 当前软件证据 | 下一层组合验收 |
|---|---|---|
| Z8 | 3.10 原码识别、MF/EXIF/光谱/VR 消费；3.12 描述符 | 实体 GM/长焦/微距组合 |
| Z9 | 5.32 ROM 描述符 | 生命周期及 AF 机身原码/实体组合 |
| Z7 II | 1.70 光谱 Thumb 消费端 | 枚举/AF 生命周期组合 |
| Z6 II | 1.70 描述符及光谱 Thumb 消费端 | 实体组合 |
| Z5 | 1.50 描述符及光谱 Thumb 消费端 | 实体组合 |
| Z30 | 1.20 描述符及光谱 Thumb 消费端 | DX、无机身位移防抖组合 |
| Z7 / Z6 | 工作区官方固件目录 | 版本绑定的描述符及状态机提取 |
| Z50 / Zfc | 工作区官方固件目录 | DX、无机身位移防抖组合 |
| Z6 III / Zf | 工作区官方固件目录 | 新世代 AF 与 MF 显示链 |
| Z50 II | 官方产品与固件目录 | DX 机身状态机 |
| Z5 II | 官方产品与固件目录 | 生命周期与 AF |
| Z5IIC | 官方目录新增条目 | 获取对应固件并绑定协议证据 |
| ZR | 官方 Z CINEMA 条目 | 视频连续对焦/光圈与关机合同 |

E6 样本 88 条 descriptor，E7 样本 166 条；前者在六份已提取 ROM 中与 E7 公共命令长度一致。164 条双窗 sweep 对应去掉两条 TX=0 项的集合。压力测试中的 descriptor replay 表示按该 ROM 的窗口合同驱动共享核心，NATIVE 标记另由原码执行取得。
Z CINEMA 页面同时列出 RED Z Mount 机型；其机身协议与生命周期作为独立生态扩展项。

## 8. 转接环对照

| 项目 | ETZ21 0204 / 0207 | TZE-02 1.8.0 | BORYOZA V120 | 本实现 V2.23 |
|---|---|---|---|---|
| 平台 | APM32F103，0207 引脚全表 | ATSAMD21E17A | HC32F460 原厂二进制 | 同硬件 C17 |
| Z 主通道 | SPI1 slave | SERCOM0 SPI slave | SPI3 | SPI3 DMA1 双窗 |
| E 通道 | USART2 | SERCOM2 async | USART1 | USART1 DMA2 |
| Z 辅助同步 | USART1 同步输出 | SERCOM1 SPI master，物理对象按资料核对 | Hotline | SPI1 42B |
| 数据侧重点 | 261 项基础身份表、能力/驱动范本 | 182 项距离查找 | 冷启动与 BL 真值 | 275 项目录、57 光谱、25 组 3D CMD 0x24、全时闭环齐焦 |
| 升级 | 版本对应 updater / SPI | 64B HID report、1024B 对齐 | USB MSC PUB | 保留原厂 MSC BL |

ETZ21 0207 的独立测量为 50Hz PB1 帧参考，TIM4 UPDATE 结束编程低窗口；本实现采用自身 60/120Hz 时序合同。
0207 的 E 脚：PA0 CS-in、PA1 CS-out、PA2 TX、PA3 RX、PB1 VD，PB7 检测强候选；Z 脚：PA4 RDY、PA5 SCK、PA6 MISO、PA7 MOSI、PA8/PA9 辅助同步。
TZE-02 应用基址 5000、1.8.0 载荷 20464B；其 182 项距离二分表与 AF 消费链是算法对照入口。
0204 与 0207 的具体地址以各自镜像为准；当前移植中引用的 CMD01/5A 与 Sony 驱动范本绑定 0207。

## 9. 镜头生态与参数来源

附录逐项列出生成头中的 275 个 LensID、名称、画幅旗标、焦距、MFD、光谱索引、光学 profile 与端点。
镜头类别包含 Sony、Sigma、Tamron、Samyang/Rokinon、Zeiss/Batis/Loxia/Touit、Voigtlander、Viltrox、Tokina 等目录身份；Laowa、7Artisans、TTArtisan、Meike、SongRaw、Thypoch、Sirui 的逆向资料另由主索引关联。

| 族 | 运行策略/数据 |
|---|---|
| Sony 807B / 8053 Gen-2 | DS13、CMD5A、1D 步进/落点、22B idle |
| Sigma C56E | 13 焦段凸轮、8 温区限速、光谱与 OIS；高速能力满足时 120Hz |
| Sigma RX63T 世代 | 能力门控、60Hz、标准 1D/3C |
| Sigma Macro / Fisheye | 倍率与近摄灵敏度；鱼眼畸变策略 |
| Tamron | 60Hz、session mode、动态 29/32B、13 个 Z profile |
| Samyang | 动态 ID03，35–150 曲线移植，标准 AF 路径 |
| Batis | ID03 光圈速度字段与 MF 环反馈 |
| 通用电子镜头 | CMD07/08/3F 实时身份、能力、焦距与端点 |
| 无电子镜头/空环 | 固定手动身份、距离显示及机身菜单合同 |

跨卡口文件表示数据来源，实际接入端固定为 E 卡口。MFD 与凸轮的目录默认值和 ROM 指定值在附录区分；同名镜头的不同固件或卡口按 LensID/版本建立映射。

A084 (`C149`) 按[官方产品规格](https://www.tamron.com/global/consumer/lenses/a084/)为 **全画幅 12–20mm F2.8 VXD**，广角端 MOD 0.18m、长端 0.28m；目录全幅标志、型号名称与缺省近摄锚点据此修订。Sony E 名称的 APS-C 前缀识别同时覆盖带厂商前缀的 `Sony E ...`。

Touit `C031/C032/C033` 按 ZEISS 系列规格申报 DX。防抖名称识别按完整 `OS/OSS/VC` 标记匹配；Tamron `C133/C134/C135` 的 `OSD` 为对焦马达名称，TX08 OIS=0 时使用 `00/44` 能力组合。

### 9.1 工作区副厂原包与研究覆盖

| 厂商/型号 | 已有资料层 | 接入设计与资料入口 |
|---|---|---|
| Laowa 10/2.8、12/2.8、15/4.5 2×、15/5 Cookie、180/4.5 1.5×、200/2 | E/Z/EF/X 原包与容器研究 | 电子光圈、手动/自动对焦按实时能力 |
| 7Artisans 10/2.8、35/1.8、40/2.5、50/1.8、85/1.8、135/1.8 | 10 个原包，E/X/Z/L 容器与版本表 | 以 E 握手核实 LensID/组掩码 |
| TTArtisan AF 17/1.8、35/1.8 II、75/2 等 | 26 个原包，bin/fir 多分发体系 | E 侧身份按 CMD07/3F，其他卡口资料作光学/容器对照 |
| Meike 24/1.4、85/1.8 II/SE/PRO、85/1.4；EFTE/EFTZ/ETZ 环 | 14 个包，跨 E/X/Z/L 与转接环容器矩阵 | E 镜头走通用能力交集 |
| SongRaw 50/1.2，FE/M 系及 Z 系 | 8 个版本容器、跨版本结构图 | 接入参数由实时握手与遥测取得 |
| Thypoch Voyager 24–50/2.8 FE | V1.0.4 / V1.0.6 / V1.3 原包与变更记录 | AF/DMF、变焦光圈以线协议回归 |
| Sirui Aurora 35/1.4 E/Z/L | E 1.0.7/1.1.0、Z 1.0.6/1.0.7、L 1.4/1.6 | L 口代码镜像与 E/Z 容器分别建档；E 接入走通用能力交集 |
| Viltrox 20/2.8 Z、50/2 Z 与 E-Z/NF 系环 | 18 个原包与对应容器档 | E 镜头目录、DX overrides 与未知型号路径 |

镜头目录数量按附录 A 的 `275` 项唯一 `LensID` 计算。各族的实体组合验证沿 `01/07/08 → ID05/06 → AF/iris → park/wake` 同一入口进行。

## 10. 构建、升级与复核入口

在仓库根目录或 `se-nz/` 下执行标准构建与主机闭环测试：

```sh
make clean && make test
python3 se-nz/tools/flash_usb_bor.py --pub se-nz/build/BORYOZA_V223.PUB
python3 se-nz/tools/dump_usb_blackbox.py --out telemetry.json
python3 se-nz/tools/decode_blackbox.py --image flash_or_disk.bin --mode auto --out decoded.json
```

内部构建产物默认生成 `se-nz/build/BORYOZA_V200.{elf,bin,PUB}` 并复制为 `se-nz/build/BORYOZA_V223.PUB`（`.readme_ver = 17020b1a` / `LF 2.23`）。
容器头 `PRODUCTVER:1.001`，16 B 头后为 APP 载荷，尾部以 `0xFF` 补齐至 `512 B` 扇区对齐，写入时校验后续扇区边界零头碰撞并保留低 `64 KiB` 原厂 Bootloader。

关键历史订正：正常 Z 主链采用 SPI+RDY；旧 exXXXX 异步通道分析归其内部诊断/监控职责。运行 VTOR 使用 APP Flash 起址。版本、针脚、相位与光谱网格统一按本版源码及对应 ROM 证据取值。

<!-- GENERATED_APPENDICES -->

## 附录 A：275 项编译目录

MFD 栏为 mm；`指定` 表示生成器的显式条目，`估算` 表示焦距公式缺省值。光谱 `−1` 表示中性回退，Z profile `−1` 表示通用光学转换。flags 为目录位掩码：01 DX、02 Macro、04 Fisheye、08 OIS、10 OIS 开关、20 高速候选、40 双浮动、80 ID03 32B；最终能力还结合实时握手。

|E LensID|名称|焦距 mm|flags|MFD/来源|光谱|Z profile|端点 E∞→Enear|
|---|---|---|---|---|---|---|---|
|0001|LA-EA1 or MC-11 Adapter|50–50|00|350/估算|-1|-1|0→0|
|0002|LA-EA2 Adapter|50–50|00|350/估算|-1|-1|0→0|
|0003|LA-EA3 Adapter|50–50|00|350/估算|-1|-1|0→0|
|0006|LA-EA4 Adapter|50–50|00|350/估算|-1|-1|0→0|
|0007|LA-EA5 Adapter|50–50|00|350/估算|-1|-1|0→0|
|000D|SAMYANG AF 35-150mm F2-2.8 FE|35–150|00|330–850/指定|56|-1|0→0|
|000F|SAMYANG V-AF 100mm T2.3 FE|100–100|00|650/指定|-1|-1|0→0|
|0015|SAMYANG AF 14-24mm F2.8 FE|14–24|00|180/指定|-1|-1|0→0|
|0016|SAMYANG AF 24-60mm F2.8 FE|24–60|00|280–320/指定|-1|-1|0→0|
|0019|SAMYANG AF 60-180mm F2.8 FE|60–180|40|600–850/指定|-1|-1|16384→26800|
|002C|Metabones Adapter|50–50|00|350/估算|-1|-1|0→0|
|004E|Metabones or Other Adapter|50–50|00|350/估算|-1|-1|0→0|
|00B8|Metabones EF S-Booster Ultra|50–50|00|350/估算|-1|-1|0→0|
|00EA|Metabones EF Mark IV|50–50|00|350/估算|-1|-1|0→0|
|00EF|Metabones EF Speed Booster|50–50|00|350/估算|-1|-1|0→0|
|6011|LA-EA4r MonsterAdapter|50–50|00|350/估算|-1|-1|0→0|
|8010|Sony E 16mm F2.8|16–16|21|200/估算|-1|-1|0→0|
|8011|Sony E 18-55mm F3.5-5.6 OSS|18–55|39|250/指定|-1|-1|0→0|
|8012|Sony E 55-210mm F4.5-6.3 OSS|55–210|39|1000/指定|-1|-1|0→0|
|8013|Sony E 18-200mm F3.5-6.3 OSS|18–200|39|500/指定|-1|-1|0→0|
|8014|Sony E 30mm F3.5 Macro|30–30|23|200/估算|-1|-1|0→0|
|8015|Sony E 24mm F1.8 ZA |24–24|21|200/估算|-1|-1|0→0|
|8016|Sony E 50mm F1.8 OSS |50–50|39|350/估算|-1|-1|0→0|
|8017|Sony E 16-70mm F4 ZA OSS|16–70|39|350/指定|-1|-1|0→0|
|8018|Sony E 10-18mm F4 OSS|10–18|39|250/指定|-1|-1|0→0|
|8019|Sony E PZ 16-50mm F3.5-5.6 OSS|16–50|39|250–300/指定|-1|-1|0→0|
|801A|Sony FE 35mm F2.8 ZA|35–35|20|245/估算|-1|-1|0→0|
|801B|Sony FE 24-70mm F4 ZA OSS|24–70|38|400/指定|-1|-1|0→0|
|801C|Sony FE 85mm F1.8|85–85|20|595/估算|-1|-1|0→0|
|801D|Sony E 18-200mm F3.5-6.3 OSS LE|18–200|39|500/指定|-1|-1|0→0|
|801E|Sony E 20mm F2.8|20–20|21|200/估算|-1|-1|0→0|
|801F|Sony E 35mm F1.8 OSS|35–35|39|245/估算|-1|-1|0→0|
|8020|Sony E PZ 18-105mm F4 G OSS|18–105|39|450–950/指定|-1|-1|0→0|
|8021|Sony FE 12-24mm F4 G|12–24|20|280/指定|-1|13|0→0|
|8022|Sony FE 90mm F2.8 Macro G OSS|90–90|3A|270/估算|-1|-1|0→0|
|8023|Sony E 18-50mm F4-5.6|18–50|21|250/指定|-1|-1|0→0|
|8024|Sony FE 24mm F1.4 GM|24–24|20|200/估算|-1|-1|0→0|
|8025|Sony FE 24-105mm F4 G OSS|24–105|38|380/指定|-1|-1|0→0|
|8027|Sony E PZ 18-200mm F3.5-6.3 OSS|18–200|39|500–800/指定|-1|-1|0→0|
|8028|Sony FE 55mm F1.8 ZA|55–55|20|385/估算|-1|-1|0→0|
|802A|Sony FE 70-200mm F4 G OSS|70–200|38|1000–1500/指定|-1|-1|0→0|
|802B|Sony FE 16-35mm F4 ZA OSS|16–35|38|280/指定|-1|-1|0→0|
|802C|Sony FE 50mm F2.8 Macro|50–50|22|200/估算|-1|-1|0→0|
|802D|Sony FE 28-70mm F3.5-5.6 OSS|28–70|38|300–450/指定|-1|-1|0→0|
|802E|Sony FE 35mm F1.4 ZA|35–35|20|245/估算|-1|-1|0→0|
|802F|Sony FE 24-240mm F3.5-6.3 OSS|24–240|38|500–800/指定|-1|-1|0→0|
|8030|Sony FE 28mm F2|28–28|20|200/估算|-1|-1|0→0|
|8031|Sony FE PZ 28-135mm F4 G OSS|28–135|38|400–950/指定|-1|-1|0→0|
|8033|Sony FE 100mm F2.8 STF GM OSS|100–100|38|700/估算|-1|-1|0→0|
|8034|Sony E PZ 18-110mm F4 G OSS|18–110|39|450–950/指定|-1|-1|0→0|
|8035|Sony FE 24-70mm F2.8 GM|24–70|20|380/指定|-1|-1|0→0|
|8036|Sony FE 50mm F1.4 ZA|50–50|20|350/估算|-1|-1|0→0|
|8038|Sony FE 50mm F1.8|50–50|20|350/估算|-1|-1|0→0|
|803A|Sony FE 21mm F2.8 (SEL28F20 + SEL075UWC)|21–21|20|200/估算|-1|-1|0→0|
|803B|Sony FE 16mm F3.5 Fisheye (SEL28F20 + SEL057FEC)|16–16|24|200/估算|-1|-1|0→0|
|803C|Sony FE 70-300mm F4.5-5.6 G OSS|70–300|38|900/指定|-1|-1|0→0|
|803D|Sony FE 100-400mm F4.5-5.6 GM OSS|100–400|38|980/指定|-1|-1|0→0|
|803E|Sony FE 70-200mm F2.8 GM OSS|70–200|38|960/指定|-1|-1|0→0|
|803F|Sony FE 16-35mm F2.8 GM|16–35|20|280/指定|-1|14|0→0|
|8050|Sony FE 400mm F2.8 GM OSS|400–400|38|2700/指定|-1|-1|0→0|
|8051|Sony E 18-135mm F3.5-5.6 OSS|18–135|39|450/指定|-1|-1|0→0|
|8052|Sony FE 135mm F1.8 GM|135–135|20|945/估算|-1|-1|0→0|
|8053|Sony FE 200-600mm F5.6-6.3 G OSS|200–600|38|2400/指定|-1|-1|0→0|
|8054|Sony FE 600mm F4 GM OSS|600–600|38|4500/指定|-1|-1|0→0|
|8055|Sony E 16-55mm F2.8 G|16–55|21|330/指定|-1|-1|0→0|
|8056|Sony E 70-350mm F4.5-6.3 G OSS|70–350|39|1100–1500/指定|-1|-1|0→0|
|8057|Sony FE C 16-35mm T3.1 G|16–35|20|280/指定|-1|-1|0→0|
|805A|Sony FE 35mm F1.8|35–35|20|220/指定|-1|-1|0→0|
|805B|Sony FE 20mm F1.8 G|20–20|20|200/估算|-1|-1|0→0|
|805C|Sony FE 12-24mm F2.8 GM|12–24|20|280/指定|-1|15|0→0|
|805E|Sony FE 50mm F1.2 GM|50–50|20|400/指定|-1|-1|0→0|
|805F|Sony FE 14mm F1.8 GM|14–14|20|200/估算|-1|-1|0→0|
|8060|Sony FE 28-60mm F4-5.6|28–60|20|300–450/指定|-1|-1|0→0|
|8061|Sony FE 35mm F1.4 GM|35–35|20|245/估算|-1|-1|0→0|
|8062|Sony FE 24mm F2.8 G|24–24|20|200/估算|-1|-1|0→0|
|8063|Sony FE 40mm F2.5 G|40–40|20|280/估算|-1|-1|0→0|
|8064|Sony FE 50mm F2.5 G|50–50|20|350/估算|-1|-1|0→0|
|8067|Sony FE PZ 16-35mm F4 G|16–35|20|280–240/指定|-1|-1|0→0|
|8069|Sony E PZ 10-20mm F4 G|10–20|21|170/指定|-1|-1|0→0|
|806A|Sony FE 70-200mm F2.8 GM OSS II|70–200|38|400–820/指定|-1|16|0→0|
|806B|Sony FE 24-70mm F2.8 GM II|24–70|20|210–300/指定|-1|-1|0→0|
|806C|Sony E 11mm F1.8|11–11|21|200/估算|-1|-1|0→0|
|806D|Sony E 15mm F1.4 G|15–15|21|200/估算|-1|-1|0→0|
|806E|Sony FE 20-70mm F4 G|20–70|20|300–250/指定|-1|-1|0→0|
|806F|Sony FE 50mm F1.4 GM|50–50|20|350/估算|-1|-1|0→0|
|8070|Sony FE 16mm F1.8 G|16–16|20|200/估算|-1|-1|0→0|
|8071|Sony FE 24-50mm F2.8 G|24–50|20|190–300/指定|-1|-1|0→0|
|8072|Sony FE 16-25mm F2.8 G|16–25|20|180–240/指定|-1|-1|0→0|
|8074|Sony FE 70-200mm F4 Macro G OSS II|70–200|3A|260–420/指定|-1|-1|0→0|
|8075|Sony FE 16-35mm F2.8 GM II|16–35|20|220/指定|-1|17|0→0|
|8076|Sony FE 300mm F2.8 GM OSS|300–300|38|2100/估算|-1|-1|0→0|
|8077|Sony E PZ 16-50mm F3.5-5.6 OSS II|16–50|39|250–300/指定|-1|-1|0→0|
|8078|Sony FE 85mm F1.4 GM II|85–85|20|595/估算|-1|-1|0→0|
|8079|Sony FE 28-70mm F2 GM|28–70|20|380/指定|-1|18|0→0|
|807A|Sony FE 400-800mm F6.3-8 G OSS|400–800|38|1700–3500/指定|-1|-1|0→0|
|807B|Sony FE 50-150mm F2 GM|50–150|20|400–740/指定|-1|12|0→0|
|807D|Sony FE 100mm F2.8 Macro GM OSS|100–100|3A|300/估算|-1|-1|0→0|
|807F|Sony FE 100-400mm F4.5 GM OSS|100–400|38|800/指定|-1|-1|0→0|
|8130|Sony FE 70-200mm F2.8 GM OSS + 1.4X Teleconverter|98–280|38|960/指定|-1|-1|0→0|
|8131|Sony FE 70-200mm F2.8 GM OSS + 2X Teleconverter|140–400|38|960/指定|-1|-1|0→0|
|8134|Sony FE 100mm F2.8 STF GM OSS (macro mode)|100–100|3A|570/指定|-1|-1|0→0|
|8135|Sony FE 100-400mm F4.5-5.6 GM OSS + 1.4X Teleconverter|140–560|38|980/指定|-1|-1|0→0|
|8136|Sony FE 100-400mm F4.5-5.6 GM OSS + 2X Teleconverter|200–800|38|980/指定|-1|-1|0→0|
|8137|Sony FE 400mm F2.8 GM OSS + 1.4X Teleconverter|560–560|38|2700/指定|-1|-1|0→0|
|8138|Sony FE 400mm F2.8 GM OSS + 2X Teleconverter|800–800|38|2700/指定|-1|-1|0→0|
|8139|Sony FE 200-600mm F5.6-6.3 G OSS + 1.4X Teleconverter|280–840|38|2400/指定|-1|-1|0→0|
|813A|Sony FE 200-600mm F5.6-6.3 G OSS + 2X Teleconverter|400–1200|38|2400/指定|-1|-1|0→0|
|813B|Sony FE 600mm F4 GM OSS + 1.4X Teleconverter|840–840|38|4500/指定|-1|-1|0→0|
|813C|Sony FE 600mm F4 GM OSS + 2X Teleconverter|1200–1200|38|4500/指定|-1|-1|0→0|
|813D|Sony FE 70-200mm F2.8 GM OSS II + 1.4X Teleconverter|98–280|38|400–820/指定|-1|19|0→0|
|813E|Sony FE 70-200mm F2.8 GM OSS II + 2X Teleconverter|140–400|38|400–820/指定|-1|20|0→0|
|813F|Sony FE 70-200mm F4 Macro G OSS II + 1.4X Teleconverter|98–280|3A|260–420/指定|-1|-1|0→0|
|8140|Sony FE 70-200mm F4 Macro G OSS II + 2X Teleconverter|140–400|3A|260–420/指定|-1|-1|0→0|
|8141|Sony FE 300mm F2.8 GM OSS + 1.4X Teleconverter|420–420|38|2000/指定|-1|-1|0→0|
|8142|Sony FE 300mm F2.8 GM OSS + 2X Teleconverter|600–600|38|2000/指定|-1|-1|0→0|
|8143|Sony FE 400-800mm F6.3-8 G OSS + 1.4X Teleconverter|560–1120|38|1700–3500/指定|-1|-1|0→0|
|8144|Sony FE 400-800mm F6.3-8 G OSS + 2X Teleconverter|800–1600|38|1700–3500/指定|-1|-1|0→0|
|8145|Sony FE 100mm F2.8 Macro GM OSS + 1.4X Teleconverter|140–140|3A|260/指定|-1|-1|0→0|
|8146|Sony FE 100mm F2.8 Macro GM OSS + 2X Teleconverter|200–200|3A|260/指定|-1|-1|0→0|
|8147|Sony FE 100-400mm F4.5 GM OSS + 1.4X Teleconverter|140–560|38|800/指定|-1|-1|0→0|
|8148|Sony FE 100-400mm F4.5 GM OSS + 2X Teleconverter|200–800|38|800/指定|-1|-1|0→0|
|C031|Zeiss Touit 12mm F2.8|12–12|01|200/估算|-1|-1|0→0|
|C032|Zeiss Touit 32mm F1.8|32–32|01|224/估算|-1|-1|0→0|
|C033|Zeiss Touit 50mm F2.8 Macro|50–50|03|200/估算|-1|-1|0→0|
|C040|Zeiss Batis 25mm F2|25–25|00|200/估算|-1|-1|0→0|
|C041|Zeiss Batis 85mm F1.8|85–85|00|595/估算|-1|-1|0→0|
|C042|Zeiss Batis 18mm F2.8|18–18|00|200/估算|-1|-1|0→0|
|C043|Zeiss Batis 135mm F2.8|135–135|00|945/估算|-1|-1|0→0|
|C044|Zeiss Batis 40mm F2 CF|40–40|02|200/估算|-1|-1|0→0|
|C050|Zeiss Loxia 50mm F2|50–50|00|350/估算|-1|-1|0→0|
|C051|Zeiss Loxia 35mm F2|35–35|00|245/估算|-1|-1|0→0|
|C052|Zeiss Loxia 21mm F2.8|21–21|00|200/估算|-1|-1|0→0|
|C053|Zeiss Loxia 85mm F2.4|85–85|00|595/估算|-1|-1|0→0|
|C054|Zeiss Loxia 25mm F2.4|25–25|00|200/估算|-1|-1|0→0|
|C131|Tamron 28-75mm F2.8 Di III RXD A036|28–75|00|190–390/指定|-1|-1|0→0|
|C132|Tamron 17-28mm F2.8 Di III RXD A046|17–28|00|190–260/指定|-1|-1|0→0|
|C133|Tamron 35mm F2.8 Di III OSD M1:2|35–35|82|200/估算|-1|-1|0→0|
|C134|Tamron 24mm F2.8 Di III OSD M1:2|24–24|82|200/估算|-1|-1|0→0|
|C135|Tamron 20mm F2.8 Di III OSD M1:2|20–20|82|200/估算|-1|-1|0→0|
|C136|Tamron 70-180mm F2.8 Di III VXD|70–180|C0|850/指定|-1|-1|14600→28200|
|C137|Tamron 28-200mm F2.8-5.6 Di III RXD|28–200|80|190–800/指定|-1|-1|0→0|
|C138|Tamron 70-300mm F4.5-6.3 Di III RXD|70–300|00|800–1500/指定|0|0|0→0|
|C139|Tamron 17-70mm F2.8 Di III-A VC RXD|17–70|09|190–390/指定|10|10|0→0|
|C13B|Tamron 11-20mm F2.8 Di III-A RXD|11–20|01|150–240/指定|-1|-1|0→0|
|C13C|Tamron 18-300mm F3.5-6.3 Di III-A VC VXD|18–300|09|150–990/指定|9|9|0→0|
|C13D|Tamron 35-150mm F2-F2.8 Di III VXD|35–150|00|330–850/指定|1|1|15000→27800|
|C13E|Tamron 28-75mm F2.8 Di III VXD G2|28–75|00|180–380/指定|3|3|0→0|
|C13F|Tamron 50-400mm F4.5-6.3 Di III VC VXD|50–400|18|250–1500/指定|6|6|0→0|
|C140|Tamron 20-40mm F2.8 Di III VXD|20–40|00|170–290/指定|2|2|0→0|
|C141|Tamron 17-50mm F4 Di III VXD A068|17–50|00|190–300/指定|-1|-1|0→0|
|C142|Tamron 70-180mm F2.8 Di III VC VXD G2 A065|70–180|08|300–850/指定|5|5|15200→28400|
|C143|Tamron 50-300mm F4.5-6.3 Di III VC VXD A069|50–300|08|220–900/指定|-1|-1|0→0|
|C144|Tamron 28-300mm F4-7.1 Di III VC VXD A074|28–300|08|190–990/指定|-1|-1|0→0|
|C145|Tamron 90mm F2.8 Di III MACRO VXD F072|90–90|42|230/指定|11|11|14800→29600|
|C146|Tamron 16-30mm F2.8 Di III VXD G2 A064|16–30|00|190–300/指定|4|4|0→0|
|C147|Tamron 25-200mm F2.8-5.6 Di III VXD G2 A075|25–200|00|160–800/指定|-1|-1|0→0|
|C148|Tamron 35-100mm F2.8 Di III VXD A078|35–100|00|220–650/指定|7|7|0→0|
|C149|Tamron 12-20mm F2.8 Di III VXD A084|12–20|00|180–280/指定|8|8|0→0|
|C230|Tokina FiRIN 20mm F2 FE AF|20–20|20|200/估算|-1|-1|0→0|
|C231|Tokina FiRIN 100mm F2.8 FE MACRO|100–100|22|300/估算|-1|-1|0→0|
|C232|Tokina atx-m 11-18mm F2.8 E|11–18|01|200/估算|-1|-1|0→0|
|C530|Sigma 30mm F1.4 DC DN \| C|30–30|01|210/估算|31|-1|0→0|
|C531|Sigma 50mm F1.4 DG HSM \| A|50–50|00|350/估算|-1|-1|0→0|
|C532|Sigma 18-300mm F3.5-6.3 DC MACRO OS HSM \| C + MC-11|18–300|1B|390/指定|-1|-1|0→0|
|C533|Sigma 18-35mm F1.8 DC HSM \| A + MC-11|18–35|01|280/指定|-1|-1|0→0|
|C534|Sigma 24-35mm F2 DG HSM \| A + MC-11|24–35|00|280/指定|-1|-1|0→0|
|C535|Sigma 24mm F1.4 DG HSM \| A + MC-11|24–24|00|200/估算|-1|-1|0→0|
|C536|Sigma 150-600mm F5-6.3 DG OS HSM \| C + MC-11|150–600|18|2800/指定|-1|-1|0→0|
|C537|Sigma 20mm F1.4 DG HSM \| A + MC-11|20–20|00|200/估算|-1|-1|0→0|
|C538|Sigma 35mm F1.4 DG HSM \| A|35–35|00|300/指定|-1|-1|3863→8988|
|C539|Sigma 150-600mm F5-6.3 DG OS HSM \| S + MC-11|150–600|18|2600/指定|-1|-1|0→0|
|C53A|Sigma 120-300mm F2.8 DG OS HSM \| S + MC-11|120–300|18|1500–2500/指定|-1|-1|0→0|
|C53C|Sigma 24-105mm F4 DG OS HSM \| A + MC-11|24–105|18|450/指定|-1|-1|0→0|
|C53D|Sigma 17-70mm F2.8-4 DC MACRO OS HSM \| C + MC-11|17–70|1B|220/指定|-1|-1|0→0|
|C53F|Sigma 50-100mm F1.8 DC HSM \| A + MC-11|50–100|01|950/指定|-1|-1|0→0|
|C543|Sigma 85mm F1.4 DG HSM \| A|85–85|00|595/估算|-1|-1|0→0|
|C545|Sigma 100-400mm F5-6.3 DG OS HSM \| C + MC-11|100–400|18|1600/指定|-1|-1|0→0|
|C547|Sigma 16mm F1.4 DC DN \| C|16–16|01|200/估算|32|-1|0→0|
|C54B|Sigma 105mm F1.4 DG HSM \| A|105–105|00|1000/指定|-1|-1|4096→12472|
|C54C|Sigma 56mm F1.4 DC DN \| C|56–56|01|392/估算|33|-1|0→0|
|C550|Sigma 70-200mm F2.8 DG OS HSM \| S + MC-11|70–200|18|1200/指定|-1|-1|0→0|
|C551|Sigma 70mm F2.8 DG MACRO \| A|70–70|02|258/指定|12|-1|4096→28296|
|C552|Sigma 45mm F2.8 DG DN \| C|45–45|20|315/估算|34|-1|0→0|
|C553|Sigma 35mm F1.2 DG DN \| A|35–35|20|245/估算|15|-1|0→0|
|C554|Sigma 14-24mm F2.8 DG DN \| A|14–24|20|280/指定|13|-1|0→0|
|C555|Sigma 24-70mm F2.8 DG DN \| A|24–70|20|180–380/指定|14|-1|0→0|
|C556|Sigma 100-400mm F5-6.3 DG DN OS \| C|100–400|38|1120–1600/指定|35|-1|0→0|
|C559|Sigma 85mm F1.4 DG DN \| A|85–85|20|595/估算|17|-1|0→0|
|C55A|Sigma 105mm F2.8 DG DN MACRO \| A|105–105|22|295/指定|16|-1|2048→18432|
|C55B|Sigma 65mm F2 DG DN \| C|65–65|20|550/指定|38|-1|0→0|
|C55C|Sigma 35mm F2 DG DN \| C|35–35|20|245/估算|37|-1|0→0|
|C55D|Sigma 24mm F3.5 DG DN \| C|24–24|20|200/估算|41|-1|0→0|
|C55E|Sigma 28-70mm F2.8 DG DN \| C|28–70|20|190–380/指定|42|-1|0→0|
|C55F|SIGMA 150-600mm F5-6.3 DG DN OS \| Sports 021|150–600|38|580–2800/指定|50|-1|10293→24052|
|C560|Sigma 35mm F1.4 DG DN \| A|35–35|20|300/指定|18|-1|4096→15143|
|C561|Sigma 90mm F2.8 DG DN \| C|90–90|20|630/估算|43|-1|0→0|
|C562|Sigma 24mm F2 DG DN \| C|24–24|20|200/估算|40|-1|0→0|
|C563|Sigma 18-50mm F2.8 DC DN \| C|18–50|21|121–300/指定|39|-1|0→0|
|C564|Sigma 20mm F2 DG DN \| C|20–20|20|200/估算|45|-1|0→0|
|C565|Sigma 16-28mm F2.8 DG DN \| C|16–28|20|250/指定|44|-1|0→0|
|C566|Sigma 20mm F1.4 DG DN \| A|20–20|20|200/估算|19|-1|0→0|
|C567|Sigma 24mm F1.4 DG DN \| A|24–24|20|200/估算|20|-1|0→0|
|C568|Sigma 60-600mm F4.5-6.3 DG DN OS \| S|60–600|38|450–2600/指定|52|-1|0→0|
|C569|Sigma 50mm F2 DG DN \| C|50–50|20|350/估算|49|-1|0→0|
|C56A|Sigma 17mm F4 DG DN \| C|17–17|20|200/估算|47|-1|0→0|
|C56B|Sigma 50mm F1.4 DG DN \| A|50–50|20|450/指定|22|-1|2890→11877|
|C56C|Sigma 14mm F1.4 DG DN \| A|14–14|20|200/估算|21|-1|0→0|
|C56D|Sigma 100-400mm F5-6.3 DG DN OS \| C 020|100–400|38|1120–1600/指定|36|-1|0→0|
|C56E|SIGMA 150-600mm F5-6.3 DG DN OS \| Sports 021|150–600|38|580–2800/指定|51|-1|10293→24052|
|C56F|Sigma 70-200mm F2.8 DG DN OS \| S|70–200|78|650–1000/指定|53|21|3200→19600|
|C570|Sigma 23mm F1.4 DC DN \| C|23–23|21|200/估算|48|-1|0→0|
|C571|Sigma 24-70mm F2.8 DG DN II \| A|24–70|20|170–340/指定|24|-1|0→0|
|C572|Sigma 500mm F5.6 DG DN OS \| S|500–500|38|3200/指定|54|-1|2880→19017|
|C573|Sigma 10-18mm F2.8 DC DN \| C|10–18|21|116–190/指定|46|-1|0→0|
|C574|Sigma 15mm F1.4 DG DN DIAGONAL FISHEYE \| A|15–15|24|385/指定|23|-1|9592→11301|
|C575|Sigma 50mm F1.2 DG DN \| A|50–50|60|400/指定|27|-1|2670→6771|
|C576|Sigma 28-105mm F2.8 DG DN \| A|28–105|20|400/指定|25|-1|0→0|
|C577|Sigma 28-45mm F1.8 DG DN \| A|28–45|20|300/指定|26|22|0→0|
|C578|Sigma 35mm F1.2 DG II \| A|35–35|60|280/指定|29|-1|2920→7539|
|C579|Sigma 300-600mm F4 DG OS \| S|300–600|38|2800–4500/指定|-1|23|0→0|
|C57A|Sigma 16-300mm F3.5-6.7 DC OS \| C|16–300|39|170–1050/指定|-1|-1|0→0|
|C57B|Sigma 12mm F1.4 DC \| C|12–12|21|200/估算|-1|-1|0→0|
|C57C|Sigma 17-40mm F1.8 DC \| A|17–40|21|280/指定|-1|24|0→0|
|C57D|Sigma 200mm F2 DG OS \| S|200–200|38|1700/指定|55|-1|0→0|
|C57E|Sigma 20-200mm F3.5-6.3 DG \| C|20–200|20|165–650/指定|-1|-1|0→0|
|C57F|Sigma 135mm F1.4 DG \| A|135–135|60|1100/指定|28|-1|3050→14336|
|C583|Sigma 35mm F1.4 DG II \| A|35–35|20|280/指定|30|-1|3050→8331|
|C584|Sigma 15mm F1.4 DC \| C|15–15|21|200/估算|-1|-1|0→0|
|C730|Voigtlander SUPER WIDE-HELIAR 15mm F4.5 III|15–15|00|200/估算|-1|-1|0→0|
|C731|Voigtlander HELIAR-HYPER WIDE 10mm F5.6|10–10|00|200/估算|-1|-1|0→0|
|C732|Voigtlander ULTRA WIDE-HELIAR 12mm F5.6 III|12–12|00|200/估算|-1|-1|0→0|
|C733|Voigtlander MACRO APO-LANTHAR 65mm F2 Aspherical|65–65|02|200/估算|-1|-1|0→0|
|C734|Voigtlander NOKTON 40mm F1.2 Aspherical|40–40|00|280/估算|-1|-1|0→0|
|C735|Voigtlander NOKTON classic 35mm F1.4|35–35|00|245/估算|-1|-1|0→0|
|C736|Voigtlander MACRO APO-LANTHAR 110mm F2.5|110–110|02|330/估算|-1|-1|0→0|
|C737|Voigtlander COLOR-SKOPAR 21mm F3.5 Aspherical|21–21|00|200/估算|-1|-1|0→0|
|C738|Voigtlander NOKTON 50mm F1.2 Aspherical|50–50|00|350/估算|-1|-1|0→0|
|C739|Voigtlander NOKTON 21mm F1.4 Aspherical|21–21|00|200/估算|-1|-1|0→0|
|C73A|Voigtlander APO-LANTHAR 50mm F2 Aspherical|50–50|00|350/估算|-1|-1|0→0|
|C73B|Voigtlander NOKTON 35mm F1.2 Aspherical SE|35–35|00|245/估算|-1|-1|0→0|
|C73E|Voigtlander APO-LANTHAR 35mm F2 Aspherical|35–35|00|245/估算|-1|-1|0→0|
|C73F|Voigtlander NOKTON 50mm F1 Aspherical|50–50|00|350/估算|-1|-1|0→0|
|C740|Voigtlander NOKTON 75mm F1.5 Aspherical|75–75|00|525/估算|-1|-1|0→0|
|C741|Voigtlander NOKTON 28mm F1.5 Aspherical|28–28|00|200/估算|-1|-1|0→0|
|C743|Voigtlander APO-LANTHAR 28mm F2 Aspherical|28–28|00|200/估算|-1|-1|0→0|
|C780|ZEISS Otus ML 50mm F1.4|50–50|00|350/估算|-1|-1|0→0|
|C781|ZEISS Otus ML 85mm F1.4|85–85|00|595/估算|-1|-1|0→0|
|C930|Samyang AF 50mm F1.4|50–50|00|350/估算|-1|-1|0→0|
|C931|Samyang AF 14mm F2.8 or Samyang AF 35mm F2.8|14–14|00|200/估算|-1|-1|0→0|
|C933|Samyang AF 35mm F1.4|35–35|00|245/估算|-1|-1|0→0|
|C934|Samyang AF 45mm F1.8|45–45|00|315/估算|-1|-1|0→0|
|C936|Samyang AF 18mm F2.8 or Samyang AF 35mm F1.8|18–18|00|200/估算|-1|-1|0→0|
|C938|Samyang AF 75mm F1.8|75–75|00|525/估算|-1|-1|0→0|
|C939|Samyang AF 35mm F1.8|35–35|00|245/估算|-1|-1|0→0|
|C93A|Samyang AF 24mm F1.8|24–24|00|200/估算|-1|-1|0→0|
|C93B|Samyang AF 12mm F2.0|12–12|01|200/估算|-1|-1|0→0|
|C93C|Samyang AF 24-70mm F2.8|24–70|00|350/指定|-1|-1|0→0|
|C93D|Samyang AF 50mm F1.4 II|50–50|00|350/估算|-1|-1|0→0|
|C93E|Samyang AF 135mm F1.8|135–135|00|945/估算|-1|-1|0→0|
|F0A0|Thypoch AF 24-50mm F2.8 FE|24–50|00|250–320/指定|-1|-1|0→0|
|F140|Viltrox 135mm F1.8 FE LAB|135–135|20|945/估算|-1|-1|0→0|
|F141|Viltrox 28mm F4.5 FE|28–28|00|200/估算|-1|-1|0→0|
|F142|Viltrox 35mm F1.2 FE LAB|35–35|20|245/估算|-1|-1|0→0|
|F143|Viltrox 85mm F1.4 FE Pro|85–85|20|595/估算|-1|-1|0→0|
|F146|Viltrox 40mm F2.5 FE Air|40–40|20|280/估算|-1|-1|0→0|
|F147|Viltrox 50mm F2.0 FE Air|50–50|20|350/估算|-1|-1|0→0|
|F148|Viltrox 25mm F1.7 E Air|25–25|01|200/估算|-1|-1|0→0|
|F150|Viltrox 50mm F1.4 FE Pro|50–50|20|350/估算|-1|-1|0→0|
|F151|Viltrox 9mm F2.8 E Air|9–9|01|200/估算|-1|-1|0→0|
|F152|Viltrox 14mm F4.0 FE Air|14–14|20|200/估算|-1|-1|0→0|
|F153|Viltrox 56mm F1.2 E Pro|56–56|01|392/估算|-1|-1|0→0|
|F154|Viltrox 85mm F2.0 FE EVO|85–85|20|595/估算|-1|-1|0→0|
|F155|Viltrox 55mm F1.8 FE EVO|55–55|20|385/估算|-1|-1|0→0|
|F157|Viltrox 15mm F1.7 E Air|15–15|01|200/估算|-1|-1|0→0|
|F15D|Viltrox 35mm F1.8 II FE EVO|35–35|20|245/估算|-1|-1|0→0|

## 附录 B：光谱索引与目录映射

|光谱索引|目录条目数|LensID|
|---|---|---|
|-1|218|0001, 0002, 0003, 0006, 0007, 000F, 0015, 0016, 0019, 002C, 004E, 00B8, 00EA, 00EF, 6011, 8010, 8011, 8012, 8013, 8014, 8015, 8016, 8017, 8018, 8019, 801A, 801B, 801C, 801D, 801E, 801F, 8020, 8021, 8022, 8023, 8024, 8025, 8027, 8028, 802A, 802B, 802C, 802D, 802E, 802F, 8030, 8031, 8033, 8034, 8035, 8036, 8038, 803A, 803B, 803C, 803D, 803E, 803F, 8050, 8051, 8052, 8053, 8054, 8055, 8056, 8057, 805A, 805B, 805C, 805E, 805F, 8060, 8061, 8062, 8063, 8064, 8067, 8069, 806A, 806B, 806C, 806D, 806E, 806F, 8070, 8071, 8072, 8074, 8075, 8076, 8077, 8078, 8079, 807A, 807B, 807D, 807F, 8130, 8131, 8134, 8135, 8136, 8137, 8138, 8139, 813A, 813B, 813C, 813D, 813E, 813F, 8140, 8141, 8142, 8143, 8144, 8145, 8146, 8147, 8148, C031, C032, C033, C040, C041, C042, C043, C044, C050, C051, C052, C053, C054, C131, C132, C133, C134, C135, C136, C137, C13B, C141, C143, C144, C147, C230, C231, C232, C531, C532, C533, C534, C535, C536, C537, C538, C539, C53A, C53C, C53D, C53F, C543, C545, C54B, C550, C579, C57A, C57B, C57C, C57E, C584, C730, C731, C732, C733, C734, C735, C736, C737, C738, C739, C73A, C73B, C73E, C73F, C740, C741, C743, C780, C781, C930, C931, C933, C934, C936, C938, C939, C93A, C93B, C93C, C93D, C93E, F0A0, F140, F141, F142, F143, F146, F147, F148, F150, F151, F152, F153, F154, F155, F157, F15D|
|0|1|C138|
|1|1|C13D|
|2|1|C140|
|3|1|C13E|
|4|1|C146|
|5|1|C142|
|6|1|C13F|
|7|1|C148|
|8|1|C149|
|9|1|C13C|
|10|1|C139|
|11|1|C145|
|12|1|C551|
|13|1|C554|
|14|1|C555|
|15|1|C553|
|16|1|C55A|
|17|1|C559|
|18|1|C560|
|19|1|C566|
|20|1|C567|
|21|1|C56C|
|22|1|C56B|
|23|1|C574|
|24|1|C571|
|25|1|C576|
|26|1|C577|
|27|1|C575|
|28|1|C57F|
|29|1|C578|
|30|1|C583|
|31|1|C530|
|32|1|C547|
|33|1|C54C|
|34|1|C552|
|35|1|C556|
|36|1|C56D|
|37|1|C55C|
|38|1|C55B|
|39|1|C563|
|40|1|C562|
|41|1|C55D|
|42|1|C55E|
|43|1|C561|
|44|1|C565|
|45|1|C564|
|46|1|C573|
|47|1|C56A|
|48|1|C570|
|49|1|C569|
|50|1|C55F|
|51|1|C56E|
|52|1|C568|
|53|1|C56F|
|54|1|C572|
|55|1|C57D|
|56|1|000D|

## 附录 C：当前 ELF 段占用

```text
se-nz/build/BORYOZA_V200.elf  :
section              size        addr
.isr_vector          4096       65536
.readme_ver             4       69632
.text              182332       69640
.ram_vectors         1024   536838144
.ramfunc             2764   536839168
.dma_sramh           9828   536848384
.data                   8   536870912
.bss                 5976   536870920
.optics_sram2       63564   536936448
.diag_sram3         16384   537001984
.retram_noload       3948   537853952
.ARM.attributes        53           0
.comment               18           0
.debug_line        195198           0
.debug_line_str       287           0
.debug_info        322914           0
.debug_abbrev       22302           0
.debug_aranges       2352           0
.debug_str          70967           0
.debug_rnglists     33555           0
.debug_loclists    221339           0
.debug_macro         8362           0
.debug_frame         8508           0
Total             1175783
```

### 当前打包结果

```json
{
  "elf": "build/BORYOZA_V200.elf",
  "bin": "build/BORYOZA_V200.bin",
  "pub": "build/BORYOZA_V200.PUB",
  "raw_bytes": 189208,
  "pub_bytes": 189440,
  "pub_sectors_512b": 370,
  "program_end_hex": "0x0003E3F0",
  "initial_msp_hex": "0x1FFFFF00",
  "reset_handler_hex": "0x00010281",
  "readme_ver_hex": "17020b1a",
  "nofpu_checked_functions_count": 107,
  "nofpu_violations": 0,
  "ram_isr_closure_functions": 42,
  "efm_sram_closure_functions": 5,
  "closure_violations": 0,
  "bin_sha256": "e6bcf0cd1037efdad17f16349df0b8301a0112bb1c50caab53872c15cfbc7190",
  "pub_sha256": "7fd7b14693d6a9dedb4e558998ab95af5eec467dccc8828f54f63bc8d436c1ec"
}
```

## 附录 D：软件验证记录

下表从 evidence 读取最近执行结果；具体输入、期望输出与变异负例随各 JSON 保存。

|入口|结果|通过/总数|
|---|---|---|
|`v223_fulltime_parfocal_report.json`|PASS|4/4|
|`v222_field_fixes_report.json`|PASS|3/3|
|`v221_field_fixes_report.json`|PASS|3/3|
|`v220_features_report.json`|PASS|6/6|
|`v219_features_report.json`|PASS|8/8|
|`v218_defocus_and_mcu_opt_report.json`|PASS|5/5|
|`v217_gm50150_confirm_green_and_sweep_report.json`|PASS|4/4|
|`v213_field_fixes_report.json`|PASS|4/4|
|`v212_gm50150_optics_report.json`|PASS|6/6|
|`m0_m2_full_chip_report.json`|PASS|78/78|
|`m1_core_z8_closed_loop.json`|PASS|53/53|
|`z8_native_recognition_v204.json`|历史构建 PASS；按本行入口重跑|9/9|
|`stress_protocol_matrix.json`|PASS|47577/47577|
|`stress_hardware.json`|PASS|19/19|
|`af_feedback_behavior.json`|PASS|23/23|
|`brand_feature_chains.json`|PARTIAL|565/565|
|`id05_selectors.json`|历史构建 PASS；按本行入口重跑|48/48|
|`five-lens-deep/feedback.json`|历史构建 MODEL_PASS_SOURCE_PENDING；按本行入口重跑|45/45|
|`five-lens-deep/mcu.json`|历史构建 FAIL；按本行入口重跑|10/15|
|`five-lens-deep/speed.json`|历史构建 PASS；按本行入口重跑|30/30|
|`five-lens-deep/endpoints.json`|历史构建 FAIL；按本行入口重跑|317/322|

### 四品牌型号与功能证据

各列为通过/总数；反馈使用有限速度模型，光学项含模板保真和旧映射诊断。源单位、真实到位窗口和实体组合按逐型号输入项接续关闭。

|LensID / 型号|AF/反馈|OS/轴分配|校正|光谱|EXIF|其他|后续输入|
|---|---|---|---|---|---|---|---|
|C56E Sigma 150-600mm F5-6.3 DG DN OS \| Sports (S021)|8/8|8/8|27/27|5/5|6/6|6/6|3|
|C56F Sigma 70-200mm F2.8 DG DN OS \| Sports (S023)|1/1|8/8|2/2|5/5|6/6|12/12|1|
|C55A Sigma 105mm F2.8 DG DN MACRO \| Art (A020)|1/1|7/7|3/3|5/5|6/6|12/12|1|
|C551 Sigma 70mm F2.8 DG Macro \| Art (A018)|1/1|7/7|3/3|5/5|6/6|6/6|2|
|C571 Sigma 24-70mm F2.8 DG DN II \| Art (A024)|1/1|7/7|2/2|5/5|6/6|12/12|1|
|C572 Sigma 500mm F5.6 DG DN OS \| Sports (S024)|1/1|8/8|3/3|5/5|5/5|12/12|3|
|C574 Sigma 15mm F1.4 DG DN Diagonal Fisheye \| Art (A024)|1/1|7/7|1/1|5/5|6/6|12/12|1|
|C576 Sigma 28-105mm F2.8 DG DN \| Art (A024)|1/1|7/7|2/2|5/5|6/6|12/12|1|
|C583 Sigma 35mm F1.4 DG II \| Art (A026)|1/1|7/7|3/3|5/5|6/6|13/13|2|
|C142 Tamron 70-180mm F/2.8 Di III VC VXD G2|0/0|8/8|5/5|5/5|6/6|5/5|1|
|C13F Tamron 50-400mm F/4.5-6.3 Di III VC VXD|0/0|8/8|5/5|5/5|6/6|5/5|1|
|C145 Tamron 90mm F/2.8 Di III MACRO VXD (1:1)|2/2|7/7|5/5|5/5|6/6|6/6|2|
|C136 Tamron 70-180mm F/2.8 Di III VXD (G1)|0/0|7/7|2/2|4/4|6/6|5/5|3|
|C149 Tamron 12-20mm F/2.8 Di III VXD|2/2|7/7|5/5|5/5|6/6|5/5|1|
|0019 AF 60-180mm F2.8 FE|0/0|7/7|2/2|4/4|6/6|5/5|2|
|000D AF 35-150mm F2-2.8 FE|0/0|7/7|2/2|5/5|6/6|5/5|1|
|0015 AF 14-24mm F2.8 FE|0/0|7/7|2/2|4/4|6/6|5/5|2|
|C044 Zeiss Batis 40mm F2 CF|0/0|7/7|3/3|4/4|6/6|6/6|3|

整体状态：**PARTIAL**；逐项输入共 31 条。完整 actual/expected、原码来源和输入清单见 `brand_feature_chains.{md,json}`。

### 启动与初始化优先验收

以下为原厂 BL、APP 指令仿真与 Z8 原码消费端组合的首窗记录，构建对应关系见上一表。测试内 `v204_*` 标签沿用历史名称，加载入口为 `build/BORYOZA_V200.bin`。

|场景|首窗 Ready ms|识别|初始化期 EFM 次数|
|---|---|---|---|
|空环|30.4|True|0|
|SCK 延迟 80ms|86.4|True|0|
|电子镜头|101.89|True|0|
|长归位镜头|139.07|True|0|

