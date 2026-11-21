#!/usr/bin/env python3
"""Read and decode BORYOZA Journal Pong (LBA 842..857), Blackbox (LBA 858..873) and JNL_A/B (LBA 874..905) over USB MSC."""

from __future__ import annotations

import argparse
import binascii
import errno
import getpass
import json
import os
import plistlib
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import decode_blackbox  # noqa: E402
import flash_usb_bor  # noqa: E402

SECTOR_BYTES = 512
DIAG_LBA_START = 842
BB_LBA_START = 858
BB_SECTORS = 16
JNL_SECTORS = 16
TOTAL_DIAG_SECTORS = 64
JNL_MAGIC = 0x425A4A4E
JNL_COMMIT = 0x434F4D54
JNL_SLOT_BYTES = 128
FX_LRU_MAGIC = 0x46584C52


def resolve_bor_device(mount_arg: Path | None) -> tuple[Path, Path, str]:
    """Resolve mounted BOR volume, prefer raw /dev/rdiskN character device, and read README.TXT."""
    mount_dir = flash_usb_bor.resolve_bor_mount(mount_arg)
    if mount_dir is None:
        raise FileNotFoundError("Mounted BOR volume not found under /Volumes")
    readme_text = flash_usb_bor.read_bor_readme(mount_dir)
    plist_raw = subprocess.check_output(["diskutil", "info", "-plist", str(mount_dir)])
    info = plistlib.loads(plist_raw)
    dev_node = str(info["DeviceNode"])
    raw_node = dev_node.replace("/dev/disk", "/dev/rdisk", 1) if dev_node.startswith("/dev/disk") else dev_node
    chosen = raw_node if Path(raw_node).exists() else dev_node
    return mount_dir, Path(chosen), readme_text


def read_diag_tail(dev_path: Path) -> bytes:
    """Read LBA 842..905 (32,768 bytes) via pure O_RDONLY without privilege escalation."""
    target_len = TOTAL_DIAG_SECTORS * SECTOR_BYTES
    try:
        fd = os.open(str(dev_path), os.O_RDONLY)
    except PermissionError:
        if sys.platform == "darwin" and str(dev_path).startswith("/dev/"):
            user = getpass.getuser()
            err = {
                "error": "PermissionDenied",
                "device": str(dev_path),
                "note": "macOS /dev/rdisk* is root:operator 0640; restart terminal/IDE session after group addition.",
                "remediation": f"sudo dseditgroup -o edit -a {user} -t user operator",
            }
            print(json.dumps(err, indent=2), file=sys.stderr)
        raise

    try:
        os.lseek(fd, DIAG_LBA_START * SECTOR_BYTES, os.SEEK_SET)
        buf = bytearray()
        while len(buf) < target_len:
            chunk = os.read(fd, target_len - len(buf))
            if not chunk:
                raise RuntimeError(f"Short read from {dev_path}: got {len(buf)}/{target_len} bytes")
            buf.extend(chunk)
        return bytes(buf)
    finally:
        os.close(fd)


def _decode_fx_lru(reserved_cfg: bytes) -> list[dict[str, Any]]:
    magic, seq, count, _pad = struct.unpack_from("<IHBB", reserved_cfg, 0)
    if magic != FX_LRU_MAGIC or count > 8:
        return []
    crc_calc = binascii.crc32(reserved_cfg[:72]) & 0xFFFFFFFF
    crc_stored = struct.unpack_from("<I", reserved_cfg, 72)[0]
    if crc_calc != crc_stored:
        return []
    out: list[dict[str, Any]] = []
    for idx in range(count):
        e_lid, f_min, av_x100, fx_en, age = struct.unpack_from("<HHHBB", reserved_cfg, 8 + (idx * 8))
        out.append({"e_lid": f"0x{e_lid:04X}", "f_min_mm": f_min, "av_x100": av_x100, "fx": fx_en, "age": age, "seq": seq})
    return out


def decode_journal_all(pong_8k: bytes, ab_16k: bytes) -> list[dict[str, Any]]:
    """Decode all 8KiB journal sectors (JNL_PONG@0x78000, JNL_A@0x7C000, JNL_B@0x7E000)."""
    sec_bytes = JNL_SECTORS * SECTOR_BYTES
    if len(pong_8k) < sec_bytes or len(ab_16k) < sec_bytes * 2:
        raise ValueError("Journal tail too short")
    records: list[dict[str, Any]] = []
    sectors = (
        ("JNL_PONG", pong_8k[:sec_bytes], 64),
        ("JNL_A", ab_16k[:sec_bytes], 64),
        ("JNL_B", ab_16k[sec_bytes : 2 * sec_bytes], 63),
    )
    for name, sec, cap in sectors:
        for slot in range(cap):
            rec = sec[slot * JNL_SLOT_BYTES : (slot + 1) * JNL_SLOT_BYTES]
            if rec == b"\xff" * JNL_SLOT_BYTES:
                continue
            magic, ver, gen, e_id, z_id, e_inf, e_near, f_mm, av_c, lmf, vr_m = struct.unpack_from(
                "<IIQHHiiHHBB", rec, 0
            )
            crc_s, commit = struct.unpack_from("<II", rec, 120)
            crc_c = binascii.crc32(rec[:120]) & 0xFFFFFFFF
            records.append(
                {
                    "sector": name,
                    "slot": slot,
                    "version": f"0x{ver:08X}",
                    "valid": (magic == JNL_MAGIC) and (commit == JNL_COMMIT) and (crc_s == crc_c),
                    "gen": gen,
                    "e_lens_id": f"0x{e_id:04X}",
                    "z_lens_id": f"0x{z_id:04X}",
                    "latched_e_inf": e_inf,
                    "latched_e_near": e_near,
                    "last_focal_mm": f_mm,
                    "last_av_c": f"0x{av_c:04X}",
                    "linear_mf_code": lmf,
                    "vr_last_mode": f"0x{vr_m:02X}",
                    "fx_lru": _decode_fx_lru(rec[34:120]),
                }
            )
    return records


def dump_telemetry(dev_path: Path, mount_dir: str = "", readme_text: str = "") -> dict[str, Any]:
    """Read and decode Blackbox and Journal sectors from a block device or disk image."""
    tail = read_diag_tail(dev_path)
    sec_bytes = JNL_SECTORS * SECTOR_BYTES
    pong_8k = tail[:sec_bytes]
    bb_8k = tail[sec_bytes : 2 * sec_bytes]
    ab_16k = tail[2 * sec_bytes :]
    blackbox = decode_blackbox.decode_blackbox_region(bb_8k)
    journal = decode_journal_all(pong_8k, ab_16k)
    return {
        "device": str(dev_path),
        "mount_dir": mount_dir,
        "readme": readme_text,
        "blackbox": blackbox,
        "journal": journal,
    }


def _pack_bb_record(ts_us: int, evt: int, lens_id: int, a0: int, a1: int, a2: int, a3: int) -> bytes:
    body = struct.pack("<IIHHIIII", decode_blackbox.BB_MAGIC, ts_us, evt, lens_id, a0, a1, a2, a3)
    return body + struct.pack("<I", binascii.crc32(body) & 0xFFFFFFFF)


def _pack_jnl_record(gen: int, e_id: int, z_id: int, e_inf: int, e_near: int, f_mm: int, av_c: int, lmf: int, vr_m: int) -> bytes:
    rec = bytearray(b"\x00" * JNL_SLOT_BYTES)
    struct.pack_into("<IIQHHiiHHBB", rec, 0, JNL_MAGIC, 2, gen, e_id, z_id, e_inf, e_near, f_mm, av_c, lmf, vr_m)
    crc = binascii.crc32(rec[:120]) & 0xFFFFFFFF
    struct.pack_into("<II", rec, 120, crc, JNL_COMMIT)
    return bytes(rec)


def run_selftest() -> int:
    """Validate synthetic 906-LBA disk image decoding end-to-end."""
    disk = bytearray(b"\xff" * (906 * SECTOR_BYTES))
    bb_off = BB_LBA_START * SECTOR_BYTES
    jnl_a_off = (BB_LBA_START + BB_SECTORS) * SECTOR_BYTES
    disk[bb_off : bb_off + 32] = _pack_bb_record(1000, 0x0001, 0x807B, 1, 2, 3, 4)
    disk[bb_off + 32 : bb_off + 64] = _pack_bb_record(2000, 0x0004, 0x807B, 0x807B, 120, 0, 0)
    disk[jnl_a_off : jnl_a_off + JNL_SLOT_BYTES] = _pack_jnl_record(7, 0x807B, 0x807B, -120, 4800, 50, 0x0300, 12, 0x02)

    with tempfile.NamedTemporaryFile(suffix=".bin") as tf:
        tmp_path = Path(tf.name)
        tmp_path.write_bytes(disk)
        out = dump_telemetry(tmp_path, mount_dir="", readme_text="SELFTEST V2.22")

    assert out["blackbox"]["valid_count"] == 2
    assert out["blackbox"]["corrupt_slots"] == []
    assert len(out["journal"]) == 1
    assert out["journal"][0]["valid"] is True
    assert out["journal"][0]["gen"] == 7
    assert out["journal"][0]["e_lens_id"] == "0x807B"
    print(json.dumps({"status": "PASS", "bb_valid": 2, "jnl_valid": 1}))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mount", type=Path, help="Optional BOR mount directory (e.g. '/Volumes/BOR     ')")
    ap.add_argument("--device", type=Path, help="Optional raw device or 906-sector disk image path")
    ap.add_argument("--out", type=Path, help="Optional output JSON file path")
    ap.add_argument("--selftest", action="store_true", help="Run synthetic 906-LBA decode selftest")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()

    try:
        if args.device is not None:
            dev_path = args.device
            mount_str = str(args.mount) if args.mount is not None else ""
            readme_text = flash_usb_bor.read_bor_readme(args.mount) if args.mount is not None else ""
        else:
            mount_dir, dev_path, readme_text = resolve_bor_device(args.mount)
            mount_str = str(mount_dir)

        telemetry = dump_telemetry(dev_path, mount_dir=mount_str, readme_text=readme_text)
    except PermissionError:
        return errno.EACCES

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(telemetry, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(telemetry, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
