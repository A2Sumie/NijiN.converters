#!/usr/bin/env python3
"""Decode 8KiB BORYOZA V2.00 Blackbox Flight Recorder (Flash 0x7A000 / BOR LBA 858..873)."""

from __future__ import annotations

import argparse
import binascii
import json
import struct
from pathlib import Path
from typing import Any

BB_MAGIC = 0x425A4242
BB_SLOT_BYTES = 32
BB_FLASH_OFFSET = 0x7A000
BB_LBA_START = 858
BB_LBA_COUNT = 16

EVENT_NAMES = {
    0x0001: "BOOT_OK",
    0x0002: "EMPTY_MOUNT_READY",
    0x0003: "E_BAUD_1M5",
    0x0004: "E_LINK_UP",
    0x0005: "E_120HZ_ACTIVE",
    0x0006: "Z_FIRST_WIN",
    0x0007: "HOTPLUG_DETACH",
    0x0008: "SHUTDOWN_PARKED",
    0x0009: "RESET_SOURCE",
    0x000A: "RESET_ROUTING",
    0x000B: "HS_IDENTITY",
    0x000C: "HS_TIMEOUT_FALLBACK",
    0x000D: "OPTICS_DISCONNECTED",
    0x000E: "DIST_FALLBACK",
    0x000F: "SESSION_SUMMARY",
    0x0010: "PERF_OPTICS_SNAPSHOT",
    0x00E1: "E_PROTO_ERR",
    0x00E2: "Z_HDR_CSUM_ERR",
    0x00E3: "AF_STALL_ABORT",
    0x00FF: "HARDFAULT",
}


def decode_event_fields(evt: int, arg0: int, arg1: int, arg2: int, arg3: int) -> dict[str, Any]:
    def s16(val: int) -> int:
        val &= 0xFFFF
        return val - 0x10000 if val >= 0x8000 else val

    if evt == 0x0004:
        return {
            "z_lens_id": f"0x{arg0 & 0xFFFF:04X}",
            "lens_family": (arg0 >> 16) & 0xFF,
            "af_class": (arg0 >> 24) & 0xFF,
            "vsync_hz": arg1 & 0xFF,
            "scale_exp_n": (arg1 >> 8) & 0xFF,
            "e_lens_ver": (arg1 >> 16) & 0xFFFF,
            "e_inf_true": s16(arg2),
            "e_near_true": s16(arg2 >> 16),
            "defocus_k_se": arg3 & 0xFFFF,
            "dist_code_x": (arg3 >> 16) & 0xFFFF,
        }
    if evt == 0x000B:
        return {
            "z_lens_id": f"0x{arg0 & 0xFFFF:04X}",
            "lens_family": (arg0 >> 16) & 0xFF,
            "af_class": (arg0 >> 24) & 0xFF,
            "f_min_mm": arg1 & 0xFFFF,
            "f_max_mm": (arg1 >> 16) & 0xFFFF,
            "optics_ready": arg2 & 0xFF,
            "hs_tx08_done": (arg2 >> 8) & 0xFF,
            "baud_kbps": (arg2 >> 16) & 0xFFFF,
            "is_apsc": arg3 & 0xFF,
            "is_macro": (arg3 >> 8) & 0xFF,
            "is_fisheye": (arg3 >> 16) & 0xFF,
            "lens_has_ois": (arg3 >> 24) & 0xFF,
        }
    if evt == 0x000C:
        return {
            "hs_step": arg0 & 0xFF,
            "cmd01_retries": (arg0 >> 8) & 0xFF,
            "hs_cmd07_ok": (arg0 >> 16) & 0xFF,
            "home_completed_ok": (arg0 >> 24) & 0xFF,
            "hs_elapsed_ticks": arg1,
            "e_rx_errors": arg2,
            "uart_baud": arg3,
        }
    if evt == 0x000D:
        return {
            "optics_ready": arg0 & 0xFF,
            "is_fisheye": (arg0 >> 8) & 0xFF,
            "is_zoom": (arg0 >> 16) & 0xFF,
            "id05_active_mask": f"0x{arg1:08X}",
            "optical_curve_valid": f"0x{arg2 & 0xFF:02X}",
            "curve_frames_seen": (arg2 >> 8) & 0xFF,
            "cmd24_read_mask": arg3 & 0xFF,
        }
    if evt == 0x000E:
        return {
            "bad_reason": arg0 & 0xFF,
            "mag_code": (arg0 >> 8) & 0xFF,
            "dist_code_x": (arg0 >> 16) & 0xFFFF,
            "focus_pos_se": s16(arg1),
            "e_inf_true": s16(arg2),
            "e_near_true": s16(arg2 >> 16),
            "cur_cz": s16(arg3) if (arg3 & 0xFFFF0000) in (0, 0xFFFF0000) else arg3,
        }
    if evt == 0x000F:
        return {
            "af_cmd_count": arg0 & 0xFFFF,
            "af_stall_count": (arg0 >> 16) & 0xFFFF,
            "e_rx_errors": arg1 & 0xFFFF,
            "z_hdr_errors": (arg1 >> 16) & 0xFFFF,
            "f_cur_mm": arg2 & 0xFFFF,
            "optics_disconnected": (arg2 >> 16) & 0xFF,
            "dist_report_invalid": (arg2 >> 24) & 0xFF,
            "last_dist_x": arg3 & 0xFFFF,
            "vsync_hz": (arg3 >> 16) & 0xFF,
            "home_completed_ok": (arg3 >> 24) & 0xFF,
        }
    if evt == 0x0010:
        return {
            "z_dist_0x117": f"0x{arg0 & 0xFFFF:04X}",
            "dist_cam_bias_117": s16(arg0 >> 16),
            "z_fnorm_0x33": arg1 & 0xFFFF,
            "z_dnorm_0x31": (arg1 >> 16) & 0xFFFF,
            "eff_focal_mm": round((arg2 & 0xFFFF) / 16.0, 2),
            "mag_a3_q14": (arg2 >> 16) & 0xFFFF,
            "first_ready_ms": round((arg3 & 0xFFFF) / 10.0, 1),
            "retram_warm_hit": (arg3 >> 16) & 0x01,
            "ds13_cache_hit": (arg3 >> 17) & 0x01,
        }
    if evt == 0x00E1:
        return {
            "wire_len": arg0 & 0xFFFF,
            "cmd_byte": f"0x{(arg0 >> 16) & 0xFF:02X}",
            "err_tag": (arg0 >> 24) & 0xFF,
            "rx_err_count": arg1,
            "hs_step": arg2 & 0xFF,
            "session_state": (arg2 >> 8) & 0xFF,
            "baud": arg3,
        }
    if evt == 0x00E2:
        return {
            "hdr0": f"0x{arg0:08X}",
            "hdr1": f"0x{arg1:08X}",
            "err_tag": arg2 & 0xFF,
            "cur_cmd": f"0x{(arg2 >> 8) & 0xFF:02X}",
            "spi3_phase": (arg2 >> 16) & 0xFF,
            "win_count": arg3,
        }
    if evt == 0x00E3:
        return {
            "last_z_af_cmd": f"0x{arg0 & 0xFF:02X}",
            "af_submode": (arg0 >> 8) & 0xFF,
            "drive_vsyncs": (arg0 >> 16) & 0xFF,
            "target_se": arg1,
            "focus_pos_se": arg2,
            "cur_cz": arg3,
        }
    if evt == 0x00FF:
        return {
            "fault_pc": f"0x{arg0:08X}",
            "fault_lr": f"0x{arg1:08X}",
            "cfsr": f"0x{arg2:08X}",
            "fault_aux_reg": f"0x{arg3:08X}",
        }
    return {}


def decode_blackbox_region(region_8k: bytes) -> dict[str, Any]:
    if len(region_8k) < BB_SLOT_BYTES:
        raise ValueError(f"Blackbox region too short: {len(region_8k)} bytes")
    records: list[dict[str, Any]] = []
    corrupt_slots: list[int] = []
    empty_slots = 0

    slot_count = len(region_8k) // BB_SLOT_BYTES
    for slot in range(slot_count):
        chunk = region_8k[slot * BB_SLOT_BYTES : (slot + 1) * BB_SLOT_BYTES]
        if chunk == b"\xFF" * BB_SLOT_BYTES:
            empty_slots += 1
            continue
        magic, ts_us, evt, lens_id, arg0, arg1, arg2, arg3, crc_stored = struct.unpack("<IIHHIIIII", chunk)
        crc_calc = binascii.crc32(chunk[:28]) & 0xFFFFFFFF
        if magic != BB_MAGIC or crc_calc != crc_stored:
            corrupt_slots.append(slot)
            continue
        rec: dict[str, Any] = {
            "slot": slot,
            "ts_us": ts_us,
            "event_code": f"0x{evt:04X}",
            "event_name": EVENT_NAMES.get(evt, f"UNKNOWN_0x{evt:04X}"),
            "lens_id": f"0x{lens_id:04X}",
            "arg0": f"0x{arg0:08X}",
            "arg1": f"0x{arg1:08X}",
            "arg2": f"0x{arg2:08X}",
            "arg3": f"0x{arg3:08X}",
            "crc32": f"0x{crc_stored:08X}",
        }
        decoded = decode_event_fields(evt, arg0, arg1, arg2, arg3)
        if decoded:
            rec["decoded"] = decoded
        records.append(rec)

    return {
        "total_slots": slot_count,
        "valid_count": len(records),
        "empty_slots": empty_slots,
        "corrupt_slots": corrupt_slots,
        "records": records,
    }


def run_selftest() -> int:
    def pack_rec(evt: int, lid: int, a0: int, a1: int, a2: int, a3: int, ts: int = 123456) -> bytes:
        body = struct.pack("<IIHHIIII", BB_MAGIC, ts, evt, lid, a0, a1, a2, a3)
        crc = binascii.crc32(body) & 0xFFFFFFFF
        return body + struct.pack("<I", crc)

    buf = bytearray(b"\xFF" * 8192)
    samples = [
        pack_rec(0x000B, 0x807B, 0x01020901, (150 << 16) | 50, 1 | (1 << 8) | (1500 << 16), 0),
        pack_rec(0x000D, 0x8050, 0x00000101, 0x0003FFFF, 0x0207, 3),
        pack_rec(0x000E, 0xF153, 2 | (0xFF << 8) | (255 << 16), 1200, (1000 & 0xFFFF) | (5000 << 16), 4096),
        pack_rec(0x000F, 0x807B, 42 | (1 << 16), 2 | (1 << 16), 85 | (1 << 16) | (2 << 24), 255 | (120 << 16) | (1 << 24)),
        pack_rec(0x0010, 0x807B, 0x5210 | ((0xFFFB & 0xFFFF) << 16), 2048 | (1024 << 16), (800 << 0) | (2457 << 16), 145 | (1 << 16) | (1 << 17)),
        pack_rec(0x00E3, 0x807B, 0xEE | (1 << 8) | (14 << 16), 9000, 3000, 8192),
        pack_rec(0x00FF, 0x807B, 0x00012345, 0x00011111, 0x00008200, 0x20028000),
    ]
    for i, rec in enumerate(samples):
        buf[i * 32 : (i + 1) * 32] = rec
    res = decode_blackbox_region(bytes(buf))
    assert res["valid_count"] == len(samples) and res["corrupt_slots"] == []
    assert res["records"][2]["decoded"]["dist_code_x"] == 255
    assert res["records"][2]["decoded"]["bad_reason"] == 2
    assert res["records"][3]["decoded"]["af_cmd_count"] == 42
    assert res["records"][4]["decoded"]["dist_cam_bias_117"] == -5
    assert res["records"][4]["decoded"]["retram_warm_hit"] == 1
    assert res["records"][4]["decoded"]["ds13_cache_hit"] == 1
    assert res["records"][6]["decoded"]["fault_aux_reg"] == "0x20028000"
    print(json.dumps({"status": "PASS", "tested_records": len(samples)}))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image", type=Path, help="512KiB Flash image, 463872B MSC disk, or 8KiB slice")
    ap.add_argument("--mode", choices=["auto", "flash512k", "lba_disk", "raw8k"], default="auto")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()
    if args.image is None:
        ap.error("--image is required unless --selftest is set")

    blob = args.image.read_bytes()
    mode = args.mode
    if mode == "auto":
        if len(blob) == 0x80000:
            mode = "flash512k"
        elif len(blob) >= (BB_LBA_START + BB_LBA_COUNT) * 512:
            mode = "lba_disk"
        else:
            mode = "raw8k"

    if mode == "flash512k":
        region = blob[BB_FLASH_OFFSET : BB_FLASH_OFFSET + 0x2000]
    elif mode == "lba_disk":
        start = BB_LBA_START * 512
        region = blob[start : start + BB_LBA_COUNT * 512]
    else:
        region = blob[:0x2000]

    summary = decode_blackbox_region(region)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
