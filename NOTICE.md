# Notice & Clean-Room Attributions

**[English](#notice--clean-room-attributions) · [简体中文](#声明与净室工程说明)**

**NijiN.converters (`BORYOZA SE-NZ`)**
Copyright 2026 A2Sumie (NijiN Project — https://github.com/A2Sumie/NijiN.converters)

All source code, linker scripts, host build utilities, and test suites in this repository are licensed under the [**Apache License, Version 2.0 (`Apache-2.0`)**](LICENSE). Any redistribution in source or binary form must retain this `NOTICE.md`, the `LICENSE` file, and prominent notices on modified files in accordance with Section 4 of the Apache License 2.0.

## Provenance, Clean-Room Scope & Trademarks

- This repository is an independent clean-room implementation targeting microcontroller adapter hardware (`HC32F460JEUA`) and electronic mount protocol translation (Sony E-mount to Nikon Z-mount).
- Camera body names, lens model names, lens IDs, and mount protocol identifiers appear strictly for hardware interoperability and user identification. All trademarks belong to their respective owners.
- This repository contains zero OEM bootloaders, zero factory camera firmware images, and zero third-party proprietary SDK headers or binaries.

## Experimental Hardware Notice

Flashing firmware to a lens mount adapter may permanently damage the adapter or interfere with attached cameras and lenses if misconfigured or interrupted. Hardware modifications and upgrades are performed at your own risk; software is provided on an "AS IS" BASIS without warranties or conditions of any kind under Sections 7 and 8 of the Apache License 2.0.

---

# 声明与净室工程说明

**NijiN.converters (`BORYOZA SE-NZ`)**
Copyright 2026 A2Sumie (NijiN Project — https://github.com/A2Sumie/NijiN.converters)

本仓库中的全部源码、链接脚本、主机打包/刷写/诊断工具与验证脚本均按 [**Apache License 2.0 (`Apache-2.0`)**](LICENSE) 开源授权。任何源码或二进制形式的再分发均须依照 Apache License 2.0 第 4 条保留 `LICENSE`、本 `NOTICE.md` 声明以及显著的修改说明。

## 净室工程边界与商标说明

- 本项目为面向微控制器转接环硬件（`HC32F460JEUA`）与电子卡口协议转换（Sony E 转 Nikon Z）的独立净室C17工程实现。
- 仓库及代码中出现的机身型号、镜头型号与卡口标识符仅用于协议互操作与设备识别，所有商标权归其各自权利人所有。
- 仓库内严格不含任何原厂 Bootloader 镜像、相机原厂固件或第三方专有 SDK 头文件与二进制文件。

## 实验性硬件风险提示

向电子镜头转接环刷写固件具有硬件损坏或通讯异常风险；所有编译、刷写与上机实验操作均由使用者自行承担风险。依据 Apache License 2.0 第 7 条与第 8 条，本软件按“现状（AS IS）”提供，不附带任何明示或暗示担保。
