#!/usr/bin/env python3
"""Pack BORYOZA_V200.elf into BORYOZA_V200.bin and BORYOZA_V200.PUB with hard static gates.

Enforces:
1. Initial SP at 0x10000 == 0x1FFFFF00 (avoids 0x1FFFFFFD..F boundary hazard)
2. Reset_Handler at 0x10004 has Thumb bit0 == 1 and lies within [0x10000, program_end)
3. README.TXT version stamp at 0x11000 == {0x17, 0x02, 0x0B, 0x1A} (V2.23 2026-11)
4. Partition isolation: Q = ceil((16 + len(raw)) / 512), program_end = 0x10000 + Q*512 - 16 <= 0x00060000
5. Zero sector-header collision: PUB[k*512 : k*512+16] != b"PRODUCTVER:1.001" for all k >= 1
6. Zero VFP in RAM/handler call closures and HAL/transport function bodies
7. EFM call closures reside in SRAM; indirect targets must resolve
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import subprocess
from pathlib import Path
from typing import Any

PUB_HEADER = b"PRODUCTVER:1.001"
README_STAMP = bytes([0x18, 0x02, 0x0B, 0x1A])
EXPECTED_MSP = 0x1FFFFF00
APP_FLASH_BASE = 0x00010000
APP_FLASH_LIMIT = 0x00060000

VFP_MNEMONIC_RE = re.compile(
    r"^\s*[0-9a-fA-F]+:\s+[0-9a-fA-F ]+\s+"
    r"(v[a-z][a-z0-9]*)(?:\.[a-z0-9]+)?\b",
    re.IGNORECASE,
)
FUNC_HDR_RE = re.compile(r"^([0-9a-fA-F]{8})\s+<([^>]+)>:$")


def collect_nofpu_obj_symbols(build_dir: Path, objdump_bin: str) -> set[str]:
    extra: set[str] = set()
    objs = (
        sorted(build_dir.glob("00_hal_*.o"))
        + sorted(build_dir.glob("01_trans_*.o"))
        + sorted(build_dir.glob("03_diag_*.o"))
        + [build_dir / "startup_hc32f460.o"]
    )
    for obj in objs:
        if not obj.is_file():
            continue
        out = subprocess.check_output([objdump_bin, "-d", str(obj)], text=True)
        for line in out.splitlines():
            m_fn = FUNC_HDR_RE.match(line.strip())
            if m_fn and m_fn.group(2) != "Reset_Handler":
                extra.add(m_fn.group(2))
    return extra


def scan_disassembly_for_vfp(
    disasm_text: str,
    extra_nofpu_names: set[str] | None = None,
) -> tuple[list[str], list[str]]:
    checked_funcs: list[str] = []
    violations: list[str] = []
    cur_func = ""
    cur_addr = 0
    must_be_nofpu = False
    obj_names = extra_nofpu_names or set()

    for line in disasm_text.splitlines():
        m_fn = FUNC_HDR_RE.match(line.strip())
        if m_fn:
            cur_addr = int(m_fn.group(1), 16)
            cur_func = m_fn.group(2)
            in_ramfunc = 0x1FFF8400 <= cur_addr < 0x1FFFA800
            is_handler = cur_func.endswith("_IRQHandler") or cur_func.endswith("_Handler")
            is_hal_or_trans = (cur_func in obj_names) or cur_func.startswith(
                (
                    "sys_clock_",
                    "gpio_",
                    "tmra1_",
                    "efm_",
                    "z_spi3_",
                    "z_spi1_",
                    "e_usart1_",
                    "e_tmra4_",
                )
            )
            must_be_nofpu = (in_ramfunc or is_handler or is_hal_or_trans) and (cur_func != "Reset_Handler")
            if must_be_nofpu:
                checked_funcs.append(f"{cur_func}@0x{cur_addr:08X}")
            continue

        if must_be_nofpu:
            m_vfp = VFP_MNEMONIC_RE.match(line)
            if m_vfp:
                violations.append(f"{cur_func} (0x{cur_addr:08X}): {line.strip()}")

    return checked_funcs, violations


def audit_ram_closure(disasm_text: str) -> dict[str, Any]:
    functions: dict[str, tuple[int, list[str]]] = {}
    current: list[str] = []
    for line in disasm_text.splitlines():
        match = FUNC_HDR_RE.match(line.strip())
        if match:
            current = []
            functions[match[2]] = (int(match[1], 16), current)
        else:
            current.append(line)
    addresses = {address: name for name, (address, _) in functions.items()}
    edges: dict[str, set[str]] = {}
    unresolved: set[str] = set()
    for name, (_, lines) in functions.items():
        body = "\n".join(lines)
        targets = set()
        for target in re.findall(r"\s+b[a-z]*(?:\.[nw])?\s+[0-9a-fA-F]+\s+<([^>+]+)(?:\+[^>]*)?>", body):
            if target in functions and target != name:
                targets.add(target)
        literals = {addresses[int(value, 16) & ~1]
                    for value in re.findall(r"\.word\s+0x([0-9a-fA-F]+)", body)
                    if (int(value, 16) & ~1) in addresses}
        targets.update(literals)
        if re.search(r"\b(?:blx|bx)\s+(?:r\d+|ip|fp)\b", body):
            unresolved.add(name)
        edges[name] = targets

    def reach(roots: set[str]) -> dict[str, list[str]]:
        paths = {name: [name] for name in roots}
        todo = list(roots)
        while todo:
            name = todo.pop()
            for target in edges[name]:
                if target not in paths:
                    paths[target] = paths[name] + [target]
                    todo.append(target)
        return paths

    roots = {name for name, (address, _) in functions.items()
             if (0x1FFF8400 <= address < 0x1FFFA800 or name.endswith("_Handler")
                 or name.endswith("_IRQHandler")) and name != "Reset_Handler"}
    paths = reach(roots)
    violations = []
    for name, path in sorted(paths.items()):
        for line in functions[name][1]:
            if VFP_MNEMONIC_RE.match(line):
                violations.append(f"{' -> '.join(path)}: {line.strip()}")
        if name in unresolved:
            violations.append(f"{' -> '.join(path)}: unresolved indirect branch")
    efm_roots = {"efm_program_word_ramfunc", "efm_erase_sector_ramfunc"} & functions.keys()
    efm_paths = reach(efm_roots)
    for name, path in sorted(efm_paths.items()):
        if not 0x1FFF8400 <= functions[name][0] < 0x1FFFA800:
            violations.append(f"EFM SRAM closure: {' -> '.join(path)}")
    return {"roots": sorted(roots), "functions": sorted(paths),
            "efm_functions": sorted(efm_paths), "violations": violations}


def pack_and_verify(
    elf_path: Path,
    bin_path: Path,
    pub_path: Path,
    objcopy_bin: str = "arm-none-eabi-objcopy",
    objdump_bin: str = "arm-none-eabi-objdump",
) -> dict[str, Any]:
    subprocess.check_call([objcopy_bin, "-O", "binary", str(elf_path), str(bin_path)])
    raw = bin_path.read_bytes()
    if len(raw) < 0x1004:
        raise ValueError(f"Binary too small ({len(raw)} bytes): missing 0x11000 .readme_ver")

    msp, reset_vec = struct.unpack_from("<II", raw, 0)
    if msp != EXPECTED_MSP:
        raise ValueError(f"Invalid initial MSP at 0x10000: 0x{msp:08X} != 0x{EXPECTED_MSP:08X}")
    if (reset_vec & 1) != 1:
        raise ValueError(f"Reset_Handler vector 0x{reset_vec:08X} missing Thumb bit0=1")
    reset_va = reset_vec & ~1
    if not (APP_FLASH_BASE <= reset_va < APP_FLASH_BASE + len(raw)):
        raise ValueError(f"Reset_Handler 0x{reset_va:08X} outside image [0x10000, 0x{APP_FLASH_BASE + len(raw):08X})")

    stamp = raw[0x1000:0x1004]
    if stamp != README_STAMP:
        raise ValueError(f"Bad .readme_ver at 0x11000: {stamp.hex()} != {README_STAMP.hex()}")

    unpadded_pub = PUB_HEADER + raw
    pad_len = (-len(unpadded_pub)) % 512
    pub = unpadded_pub + (b"\xFF" * pad_len)
    sectors = len(pub) // 512
    program_end = APP_FLASH_BASE + sectors * 512 - 16
    if program_end > APP_FLASH_LIMIT:
        raise ValueError(f"program_end 0x{program_end:08X} exceeds 0x{APP_FLASH_LIMIT:08X} FS guard")

    for k in range(1, sectors):
        if pub[k * 512 : k * 512 + 16] == PUB_HEADER:
            raise ValueError(f"Sector {k} (offset 0x{k * 512:X}) collides with PRODUCTVER:1.001 header")

    disasm = subprocess.check_output([objdump_bin, "-d", str(elf_path)], text=True)
    extra_syms = collect_nofpu_obj_symbols(elf_path.parent, objdump_bin)
    checked_funcs, vfp_violations = scan_disassembly_for_vfp(disasm, extra_syms)
    if vfp_violations:
        raise ValueError("Forbidden VFP instructions in ISR/.ramfunc/HAL/Transport:\n" + "\n".join(vfp_violations))
    closure = audit_ram_closure(disasm)
    if closure["violations"]:
        raise ValueError("RAM/ISR/EFM closure violations:\n" + "\n".join(closure["violations"]))

    pub_path.write_bytes(pub)
    manifest = {
        "elf": str(elf_path),
        "bin": str(bin_path),
        "pub": str(pub_path),
        "raw_bytes": len(raw),
        "pub_bytes": len(pub),
        "pub_sectors_512b": sectors,
        "program_end_hex": f"0x{program_end:08X}",
        "initial_msp_hex": f"0x{msp:08X}",
        "reset_handler_hex": f"0x{reset_vec:08X}",
        "readme_ver_hex": stamp.hex(),
        "nofpu_checked_functions_count": len(checked_funcs),
        "nofpu_violations": len(vfp_violations),
        "ram_isr_closure_functions": len(closure["functions"]),
        "efm_sram_closure_functions": len(closure["efm_functions"]),
        "closure_violations": len(closure["violations"]),
        "bin_sha256": hashlib.sha256(raw).hexdigest(),
        "pub_sha256": hashlib.sha256(pub).hexdigest(),
    }
    (pub_path.parent / "pack_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def run_selftest() -> None:
    fake_disasm = (
        "1fff8400 <SPI1_DMA2_TC0_IRQHandler>:\n"
        "1fff8400: ed2d 8b02 vpush {d8}\n"
        "1fff8404: 4770      bx lr\n"
    )
    _, viol = scan_disassembly_for_vfp(fake_disasm)
    assert len(viol) == 1, "Selftest failed to catch vpush inside IRQHandler"
    through_veneer = (
        "1fff8400 <test_IRQHandler>:\n"
        "1fff8400: f000 f800 bl 1fff8410 <bridge_veneer>\n"
        "1fff8410 <bridge_veneer>:\n"
        "1fff8410: f8df f000 ldr.w pc, [pc]\n"
        "1fff8414: 00011001 .word 0x00011001\n"
        "00011000 <float_core>:\n"
        "00011000: ed2d 8b02 vpush {d8}\n"
    )
    assert audit_ram_closure(through_veneer)["violations"]
    direct = through_veneer.replace("1fff8410 <bridge_veneer>", "00011000 <float_core>", 1)
    assert audit_ram_closure(direct)["violations"]
    assert audit_ram_closure(direct.replace("vpush {d8}", "vldmia r0, {d0}"))["violations"]
    outside_efm = (
        "1fff8400 <efm_program_word_ramfunc>:\n"
        "1fff8400: f000 f800 bl 00011000 <flash_helper>\n"
        "00011000 <flash_helper>:\n"
        "00011000: 4770 bx lr\n"
    )
    assert audit_ram_closure(outside_efm)["violations"]
    assert not audit_ram_closure(through_veneer.replace("vpush {d8}", "push {r4}"))["violations"]
    assert audit_ram_closure("1fff8400 <test_IRQHandler>:\n1fff8400: 4798 blx r3\n")["violations"]
    mixed_indirect = (
        "1fff8400 <test_IRQHandler>:\n"
        "1fff8400: 4798 blx r3\n"
        "1fff8402: 4770 bx lr\n"
        "1fff8404: 1fff8411 .word 0x1fff8411\n"
        "1fff8410 <integer_helper>:\n"
        "1fff8410: 4770 bx lr\n"
    )
    assert audit_ram_closure(mixed_indirect)["violations"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--elf", type=Path)
    ap.add_argument("--bin", type=Path)
    ap.add_argument("--pub", type=Path)
    ap.add_argument("--objcopy", default="arm-none-eabi-objcopy")
    ap.add_argument("--objdump", default="arm-none-eabi-objdump")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    run_selftest()
    if args.selftest and args.elf is None:
        print(json.dumps({"selftest": "PASS"}))
        return 0
    if not (args.elf and args.bin and args.pub):
        ap.error("--elf, --bin, and --pub are required")

    manifest = pack_and_verify(args.elf, args.bin, args.pub, args.objcopy, args.objdump)
    print(json.dumps(manifest))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
