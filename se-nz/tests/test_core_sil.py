#!/usr/bin/env python3
"""Standalone Host ASan/UBSan Closed-Loop SIL Verification Suite for NijiN SE-NZ V2.23.

Exercises the compiled libboryoza_core shared library (.dylib/.so built from identical
se-nz/src/02_core/*.c sources) purely through E-mount and Z-mount wire interfaces.
Requires only Python 3.10+ and the built host shared library (zero external ROMs).
"""

from __future__ import annotations

import collections
import ctypes
import json
import math
import os
import platform
import struct
import subprocess
import sys
from pathlib import Path
from typing import Any


def _ensure_macos_asan_preloaded() -> None:
    if sys.platform != "darwin" or os.environ.get("BORYOZA_ASAN_REEXEC") == "1":
        return
    dylib = Path(__file__).resolve().parents[1] / "build/libboryoza_core.dylib"
    if not dylib.is_file():
        return
    otool_out = subprocess.check_output(["otool", "-l", str(dylib)], text=True)
    asan_rt: Path | None = None
    for line in otool_out.splitlines():
        s = line.strip()
        if s.startswith("path ") and " (offset" in s:
            rpath = s.split("path ", 1)[1].split(" (offset", 1)[0].strip()
            cand = Path(rpath) / "libclang_rt.asan_osx_dynamic.dylib"
            if cand.is_file():
                asan_rt = cand
                break
    if asan_rt is not None and str(asan_rt) not in os.environ.get("DYLD_INSERT_LIBRARIES", ""):
        real_py = Path(sys.exec_prefix) / "Resources/Python.app/Contents/MacOS/Python"
        target_py = str(real_py) if real_py.is_file() else sys.executable
        env = dict(os.environ)
        prev = env.get("DYLD_INSERT_LIBRARIES", "")
        env["DYLD_INSERT_LIBRARIES"] = f"{asan_rt}:{prev}" if prev else str(asan_rt)
        env["BORYOZA_ASAN_REEXEC"] = "1"
        os.execve(target_py, [target_py, *sys.argv], env)


_ensure_macos_asan_preloaded()

SE_NZ_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = SE_NZ_ROOT / "build"
SHLIB_EXT = "dylib" if platform.system() == "Darwin" else "so"
LIB_PATH = BUILD_DIR / f"libboryoza_core.{SHLIB_EXT}"

BB_EVT_AF_STALL_ABORT = 0x00E3
SONY_CAPS = (
    0x01, 0x03, 0x04, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
    0x10, 0x16, 0x1C, 0x1D, 0x32, 0x3C, 0x3D, 0x3F, 0x41, 0x43, 0x44, 0x49, 0x4B, 0x4C, 0x5A,
)
SIGMA_CAPS = (
    0x01, 0x03, 0x04, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
    0x10, 0x16, 0x19, 0x1C, 0x1D, 0x1F, 0x3C, 0x3D, 0x3F,
)


class ETxCmd(ctypes.Structure):
    """Host mirror of e_tx_cmd_t queued from core to E-mount UART."""

    _fields_ = [
        ("cmd", ctypes.c_uint8),
        ("frame_type", ctypes.c_uint8),
        ("payload_len", ctypes.c_uint16),
        ("payload", ctypes.c_uint8 * 56),
        ("serial", ctypes.c_uint32),
    ]


class ZCmdReq(ctypes.Structure):
    """Host mirror of z_cmd_req_t submitted on Z-mount SPI3."""

    _fields_ = [
        ("cmd", ctypes.c_uint8),
        ("sub", ctypes.c_uint8),
        ("req_words", ctypes.c_uint16),
        ("mosi_buf", ctypes.POINTER(ctypes.c_uint8)),
        ("mosi_len", ctypes.c_size_t),
        ("now_ticks", ctypes.c_uint64),
    ]


def load_core_library() -> ctypes.CDLL:
    """Load libboryoza_core shared library and bind exported C signatures."""
    if not LIB_PATH.is_file():
        raise FileNotFoundError(f"Missing host shared library: {LIB_PATH}")
    lib = ctypes.CDLL(str(LIB_PATH))
    lib.lens_session_init.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.lens_session_start_mount_handshake.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint64]
    lib.lens_session_pop_tx.argtypes = [ctypes.c_void_p, ctypes.POINTER(ETxCmd)]
    lib.lens_session_pop_tx.restype = ctypes.c_bool
    lib.lens_session_on_emount_frame.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint64, ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t,
    ]
    lib.lens_session_on_emount_frame.restype = ctypes.c_bool
    lib.lens_session_build_id03_wire.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8]
    lib.lens_session_build_id03_wire.restype = ctypes.c_size_t
    lib.lens_session_build_id04_wire.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8]
    lib.lens_session_build_id04_wire.restype = ctypes.c_size_t
    lib.lens_session_pop_diag.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.lens_session_pop_diag.restype = ctypes.c_bool
    lib.z_dispatch_command_ram_safe.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ZCmdReq),
        ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
    ]
    lib.z_dispatch_command_ram_safe.restype = ctypes.c_size_t
    lib.af_servo_build_hotline_frame.argtypes = [
        ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint64,
    ]
    lib.af_servo_emit_vsync_drive.argtypes = [ctypes.c_void_p]
    lib.lens_db_unpack_spectral81.argtypes = [ctypes.c_int16, ctypes.POINTER(ctypes.c_uint8)]
    lib.lens_db_unpack_spectral81.restype = ctypes.c_bool
    lib.lens_db_lookup.argtypes = [ctypes.c_uint16]
    lib.lens_db_lookup.restype = ctypes.c_void_p
    return lib


def seal_z_frame(raw: bytearray) -> bytes:
    """Stamp Z-mount 0x55 sync byte and trailing negative-sum checksum."""
    raw[0] = 0x55
    raw[-1] = (-sum(raw[:-1])) & 0xFF
    return bytes(raw)


def wrap_emount_frame(frame_type: int, seq: int, cmd: int, payload: bytes) -> bytes:
    """Wrap an E-mount wire frame with length, checksum16, and 0x55 sentinel."""
    wlen = len(payload) + 9
    frame = bytearray([0xF0, wlen & 0xFF, (wlen >> 8) & 0xFF, frame_type & 0xFF, seq & 0xFF, cmd & 0xFF])
    frame.extend(payload)
    s16 = sum(frame[1:]) & 0xFFFF
    frame.extend([s16 & 0xFF, (s16 >> 8) & 0xFF, 0x55])
    return bytes(frame)


class CoreHarness:
    """Host harness driving se-nz core symbols strictly via wire frames."""

    def __init__(self, lib: ctypes.CDLL) -> None:
        self.lib = lib
        self.ctx = ctypes.create_string_buffer(8192)
        self.opt = ctypes.create_string_buffer(65536)
        self.ticks = 3000
        self.seq = 0
        self.lib.lens_session_init(self.ctx, self.opt)

    def start_handshake(self) -> None:
        self.lib.lens_session_start_mount_handshake(self.ctx, self.opt, ctypes.c_uint64(self.ticks))

    def pop_all_tx(self) -> list[tuple[int, bytes]]:
        out: list[tuple[int, bytes]] = []
        item = ETxCmd()
        while self.lib.lens_session_pop_tx(self.ctx, ctypes.byref(item)):
            out.append((int(item.cmd), bytes(item.payload[: item.payload_len])))
        return out

    def feed_emount_frame(self, frame_type: int, cmd: int, payload: bytes) -> bool:
        self.ticks += 1500
        self.seq = (self.seq + 1) & 0xFF
        wire = wrap_emount_frame(frame_type, self.seq, cmd, payload)
        arr = (ctypes.c_uint8 * len(wire)).from_buffer_copy(wire)
        return bool(
            self.lib.lens_session_on_emount_frame(
                self.ctx, self.opt, ctypes.c_uint64(self.ticks), arr, len(wire)
            )
        )

    def build_id03(self) -> bytes:
        buf = (ctypes.c_uint8 * 72)()
        self.seq = (self.seq + 1) % 240
        wlen = self.lib.lens_session_build_id03_wire(self.ctx, buf, ctypes.c_uint8(self.seq))
        return bytes(buf[:wlen])

    def build_id04(self) -> bytes:
        buf = (ctypes.c_uint8 * 72)()
        wlen = self.lib.lens_session_build_id04_wire(self.ctx, buf, ctypes.c_uint8(self.seq))
        return bytes(buf[:wlen])

    def z_exchange(self, cmd: int, sub: int, win_bytes: int, mosi: bytes | None = None) -> bytes:
        self.ticks += 600
        req_words = (win_bytes // 2) - 1
        mosi_ptr = ctypes.POINTER(ctypes.c_uint8)()
        mosi_len = 0
        mosi_arr = None
        if mosi is not None:
            mosi_arr = (ctypes.c_uint8 * len(mosi)).from_buffer_copy(mosi)
            mosi_ptr = ctypes.cast(mosi_arr, ctypes.POINTER(ctypes.c_uint8))
            mosi_len = len(mosi)
        req = ZCmdReq(cmd, sub, req_words, mosi_ptr, mosi_len, self.ticks)
        small = (ctypes.c_uint8 * 516)()
        direct_ptr = ctypes.POINTER(ctypes.c_uint8)()
        nbytes = self.lib.z_dispatch_command_ram_safe(
            self.ctx, self.opt, ctypes.byref(req), small, ctypes.byref(direct_ptr)
        )
        assert nbytes == win_bytes, f"Unexpected SPI3 length {nbytes} != {win_bytes}"
        if bool(direct_ptr):
            return ctypes.string_at(direct_ptr, win_bytes)
        return bytes(small[:win_bytes])

    def build_hotline(self, seq: int = 1) -> bytes:
        self.ticks += 3000
        hot = (ctypes.c_uint8 * 42)()
        self.lib.af_servo_build_hotline_frame(self.ctx, ctypes.c_uint8(seq), hot, ctypes.c_uint64(self.ticks))
        return bytes(hot)


def make_id05_108b(
    *,
    dist_x: int = 384,
    f_opt_0p1: int = 1050,
    f_nom_0p1: int = 1050,
    p_cur: int = 0x1300,
    is_mf: int = 0,
    ois_on: int = 1,
    ois_mode: int = 1,
) -> bytes:
    """Build 18-group 108B E-mount ID 0x05 status payload."""
    buf = bytearray(108)
    struct.pack_into("<HH", buf, 0, p_cur, p_cur)
    struct.pack_into("<H", buf, 9, 350)
    struct.pack_into("<H", buf, 17, p_cur)
    struct.pack_into("<H", buf, 20, dist_x)
    buf[23] = 0xFF
    struct.pack_into("<HH", buf, 24, f_opt_0p1, f_nom_0p1)
    buf[62] = 0x01 | ((is_mf & 1) << 1)
    buf[64] = ((ois_on & 1) << 4) | ((max(0, ois_mode - 1) & 3) << 5)
    return bytes(buf)


def make_id06_39b(
    pos_se: int,
    lim_far: int,
    lim_near: int,
    *,
    zoom_norm: int = 0x0100,
    moving: int = 0,
    b6_flags: int = 0x10,
) -> bytes:
    """Build 39B E-mount ID 0x06 focus/zoom telemetry payload."""
    pay = bytearray(39)
    pay[1] = moving & 0xFF
    struct.pack_into("<h", pay, 2, pos_se)
    pay[6] = b6_flags & 0xFF
    struct.pack_into("<hh", pay, 7, lim_far, lim_near)
    struct.pack_into("<HH", pay, 11, zoom_norm, 100)
    pay[15] = 25
    return bytes(pay)


def complete_mock_handshake(
    core: CoreHarness,
    lens_id: int,
    cap_cmds: tuple[int, ...],
    initial_id05: bytes,
    initial_id06: bytes,
    *,
    tx08_ois: int = 0,
    tx08_120hz: int = 1,
    lens_family: int = 2,
    drain_startup_id04: bool = True,
) -> None:
    """Drive full E-mount initialization handshake to E_SESSION_ACTIVE."""
    core.start_handshake()
    cap = bytearray(32)
    for cmd_id in cap_cmds:
        idx = cmd_id - 1
        cap[idx >> 3] |= 1 << (idx & 7)
    for _ in range(30):
        tx_list = core.pop_all_tx()
        if not tx_list:
            break
        for cmd, _pay in tx_list:
            if cmd == 0x01:
                core.feed_emount_frame(2, 0x01, bytes(cap))
            elif cmd == 0x07:
                tx07 = bytearray(35)
                tx07[0] = lens_family
                tx07[6] = 0x02
                struct.pack_into("<H", tx07, 9, lens_id)
                core.feed_emount_frame(2, 0x07, bytes(tx07))
            elif cmd == 0x0C:
                core.feed_emount_frame(2, 0x0C, b"\x01")
            elif cmd in (0x3F, 0x09, 0x0B, 0x0D):
                core.feed_emount_frame(2, cmd, b"\x00\x00")
            elif cmd == 0x3D:
                core.feed_emount_frame(2, 0x3D, bytes(64))
            elif cmd == 0x08:
                tx08 = bytearray(202)
                struct.pack_into("<HH", tx08, 0, 0x1300, 0x1900)
                tx08[0x0B] = tx08_ois
                tx08[0x0C] = 0xB2 if tx08_120hz else 0x12
                tx08[0x0D] = 0xE1
                tx08[0x18] = 0x19
                core.feed_emount_frame(2, 0x08, bytes(tx08))
            elif cmd in (0x10, 0x32):
                core.feed_emount_frame(2, cmd, b"\x00")
            elif cmd == 0x5A:
                core.feed_emount_frame(2, 0x5A, bytes(55))
            elif cmd == 0x0A:
                core.feed_emount_frame(2, 0x0A, struct.pack("<II", 0x0003FFFF, 0))
                core.feed_emount_frame(2, 0x05, initial_id05)
                core.feed_emount_frame(2, 0x06, initial_id06)
    if drain_startup_id04:
        _ = core.build_id04()
        _ = core.build_id04()


class VarifocalPlant:
    """Finite-slew, deadband-limited varifocal E-mount plant with optional dist lag."""

    def __init__(
        self,
        *,
        f_mm: float = 150.0,
        dist_m: float = 1.96,
        max_slew: int = 850,
        deadband_se: int = 3,
        is_mf: int = 0,
        ignore_1d: bool = False,
        dist_lag_frames: int = 0,
        cam_spec: tuple[float, float, int, int, int, int, float, float, float] | None = None,
    ) -> None:
        self.f_mm = f_mm
        self.max_slew = max_slew
        self.deadband_se = deadband_se
        self.is_mf = is_mf
        self.ignore_1d = ignore_1d
        self.dist_lag_frames = max(0, dist_lag_frames)
        self.cam_spec = cam_spec
        self.hw_moving = 0
        self.mf_ring_dir = 0
        self.b6_flags = 0x10
        self.f01, self.zn, self.ei, self.en, self.mfd_m, self.bias_bins = self._eval_cam(f_mm)
        d_ideal = dist_m / (2.0 ** (self.bias_bins / 64.0))
        r = max(0.0, min(1.0, self.mfd_m / d_ideal))
        self.focus_se = round(self.ei + r * (self.en - self.ei))
        self.target_se = self.focus_se
        self.true_dist_x = self._calc_dist_x()
        self._lag_q: collections.deque[int] = collections.deque(
            [self.true_dist_x] * (self.dist_lag_frames + 1), maxlen=self.dist_lag_frames + 1
        )
        self.dist_x = self._lag_q[0]

    def _eval_cam(self, f_mm: float) -> tuple[int, int, int, int, float, float]:
        if self.cam_spec is None:
            u = max(0.0, min(1.0, (f_mm - 50.0) / 100.0))
            f_min_max_ei = (7857, 8624, 12108, 21974, 0.40, 0.74, 11.06 * (1.0 - u) - 6.69 * u)
            ei_w, ei_t, en_w, en_t, mfd_w, mfd_t, bias = f_min_max_ei
        else:
            f_min, f_max, ei_w, en_w, ei_t, en_t, mfd_w, mfd_t, span = self.cam_spec
            u = max(0.0, min(1.0, (f_mm - f_min) / max(1.0, f_max - f_min)))
            bias = span * (1.0 - 2.0 * u)
        return (
            round(f_mm * 10.0),
            round(0x0100 + u * (0x3F00 - 0x0100)),
            round(ei_w + u * (ei_t - ei_w)),
            round(en_w + u * (en_t - en_w)),
            mfd_w + u * (mfd_t - mfd_w),
            bias,
        )

    def _calc_dist_x(self) -> int:
        if self.focus_se <= self.ei + self.deadband_se:
            return 0xFFFF
        r = min(1.0, max(1e-6, (self.focus_se - self.ei) / max(1, self.en - self.ei)))
        d_m = (self.mfd_m / r) * (2.0 ** (self.bias_bins / 64.0))
        return max(1, min(0x06FF, round(384.0 + 64.0 * math.log2(max(0.05, d_m)))))

    def set_zoom(self, f_mm: float) -> None:
        self.f_mm = f_mm
        self.f01, self.zn, self.ei, self.en, self.mfd_m, self.bias_bins = self._eval_cam(f_mm)
        self.focus_se = max(self.ei, min(self.en, self.focus_se))
        self.target_se = self.focus_se
        self.true_dist_x = self._calc_dist_x()
        if self.dist_lag_frames == 0:
            self._lag_q.append(self.true_dist_x)
            self.dist_x = self._lag_q[0]

    def feed(self, core: CoreHarness) -> None:
        id05 = bytearray(
            make_id05_108b(
                dist_x=self.dist_x, f_opt_0p1=self.f01, f_nom_0p1=self.f01, is_mf=self.is_mf
            )
        )
        id05[60] = self.mf_ring_dir & 0xFF
        id06 = make_id06_39b(
            self.focus_se,
            self.ei,
            self.en,
            zoom_norm=self.zn,
            moving=self.hw_moving,
            b6_flags=self.b6_flags,
        )
        core.feed_emount_frame(1, 0x05, bytes(id05) + b"\x06" + id06)

    def apply_id04(self, d4: bytes) -> None:
        if (not self.ignore_1d) and len(d4) >= 24 and d4[9] == 0x83 and d4[19] == 0x1D:
            if d4[23] == 0x00:
                self.target_se = max(self.ei, min(self.en, struct.unpack_from("<h", d4, 20)[0]))
        elif len(d4) >= 20 and d4[9] == 0x83 and d4[19] == 0x1C:
            self.target_se = self.focus_se
        diff = 0 if self.ignore_1d else (self.target_se - self.focus_se)
        if abs(diff) <= self.deadband_se:
            self.hw_moving = 0
        else:
            step = max(-self.max_slew, min(self.max_slew, diff))
            self.focus_se = max(self.ei, min(self.en, self.focus_se + step))
            self.hw_moving = 2 if step > 0 else 1
        self.true_dist_x = self._calc_dist_x()
        self._lag_q.append(self.true_dist_x)
        self.dist_x = self._lag_q[0]


def test_catalog_and_optics(lib: ctypes.CDLL) -> dict[str, Any]:
    """Verify catalog lookup, 57 unpacked spectral curves, and 3D CMD 0x24 tables."""
    out81 = (ctypes.c_uint8 * 81)()
    for idx in range(57):
        assert lib.lens_db_unpack_spectral81(idx, out81), f"Spectral unpack failed at idx={idx}"
    for lid in (0x807B, 0xC56E, 0xC56F, 0xC13D):
        assert lib.lens_db_lookup(lid), f"Missing catalog entry 0x{lid:04X}"

    core = CoreHarness(lib)
    pl = VarifocalPlant(f_mm=150.0, dist_m=2.0)
    complete_mock_handshake(
        core, 0x807B, SONY_CAPS,
        make_id05_108b(dist_x=pl.dist_x, f_opt_0p1=pl.f01, f_nom_0p1=pl.f01),
        make_id06_39b(pl.focus_se, pl.ei, pl.en, zoom_norm=pl.zn),
    )
    c10 = core.z_exchange(0x10, 0x00, 28)
    c11 = core.z_exchange(0x11, 0x00, 74)
    c12 = core.z_exchange(0x12, 0x00, 66)
    c24a = core.z_exchange(0x24, 0x00, 7426)
    c24b = core.z_exchange(0x24, 0x10, 3468)
    assert (sum(c10) & 0xFF) == 0 and c10[3:7] == bytes([0x00, 0x00, 0x23, 0x02])
    assert (sum(c11) & 0xFF) == 0 and b"50-150" in c11
    assert (sum(c12) & 0xFF) == 0 and struct.unpack_from("<H", c12, 0x2B)[0] == 50
    assert struct.unpack_from("<H", c12, 0x2F)[0] == 150
    assert (sum(c24a) & 0xFF) == 0 and (sum(c24b) & 0xFF) == 0
    return {"spectral_checked": 57, "ver_token": c10[3:7].hex(), "cmd24_checksum_ok": True}


def test_fulltime_parfocal_and_body_mf_e0(lib: ctypes.CDLL) -> dict[str, Any]:
    """Verify full-time closed-loop parfocal across modes, Body-MF E0 brake, and b6=0 settle."""
    mode_errs: dict[str, int] = {}
    for mode in ("afs", "body_mf", "barrel_mf"):
        core = CoreHarness(lib)
        is_mf = 1 if mode == "barrel_mf" else 0
        pl = VarifocalPlant(f_mm=150.0, dist_m=1.96, is_mf=is_mf)
        complete_mock_handshake(
            core, 0x807B, SONY_CAPS,
            make_id05_108b(dist_x=pl.dist_x, f_opt_0p1=pl.f01, f_nom_0p1=pl.f01, is_mf=is_mf),
            make_id06_39b(pl.focus_se, pl.ei, pl.en, zoom_norm=pl.zn),
        )
        mosi20 = bytearray(52)
        if mode == "body_mf":
            mosi20[2] = 0x06
            mosi20[0x2B] = 0x01
        core.z_exchange(0x20, 0x00, 258, mosi=seal_z_frame(mosi20))
        for _ in range(3):
            pl.feed(core)
            _ = core.build_id04()
        anchor_x = pl.true_dist_x
        for k in range(1, 16):
            pl.set_zoom(150.0 - 100.0 * (k / 15.0))
            pl.feed(core)
            pl.apply_id04(core.build_id04())
        for _ in range(12):
            pl.feed(core)
            pl.apply_id04(core.build_id04())
        err = abs(pl.true_dist_x - anchor_x)
        assert err <= 1, f"Mode {mode} parfocal error {err} > 1"
        mode_errs[mode] = err

    # Body-MF active C0 -> E0 brake must keep ID04[9] == 0x83 with 0x1C, halt plant, then settle 0x81
    c_bmf = CoreHarness(lib)
    pl_bmf = VarifocalPlant(f_mm=150.0, dist_m=1.96)
    complete_mock_handshake(
        c_bmf, 0x807B, SONY_CAPS,
        make_id05_108b(dist_x=pl_bmf.dist_x, f_opt_0p1=pl_bmf.f01, f_nom_0p1=pl_bmf.f01),
        make_id06_39b(pl_bmf.focus_se, pl_bmf.ei, pl_bmf.en, zoom_norm=pl_bmf.zn),
    )
    mosi_bmf = bytearray(52)
    mosi_bmf[2] = 0x06
    mosi_bmf[0x2B] = 0x01
    c_bmf.z_exchange(0x20, 0x00, 258, mosi=seal_z_frame(mosi_bmf))
    mosi_c0 = bytearray(24)
    struct.pack_into("<i", mosi_c0, 13, 10000)
    mosi_c0[5], mosi_c0[6], mosi_c0[12] = 0xFF, 0x3F, 0x11
    c_bmf.z_exchange(0xC0, 0x00, 24, mosi=seal_z_frame(mosi_c0))
    pl_bmf.apply_id04(c_bmf.build_id04())
    assert pl_bmf.hw_moving != 0
    pl_bmf.feed(c_bmf)
    c_bmf.z_exchange(0xE0, 0x00, 4, mosi=seal_z_frame(bytearray([0x55, 0x11, 0, 0])))
    d4_stop = c_bmf.build_id04()
    assert d4_stop[9] == 0x83 and len(d4_stop) >= 20 and d4_stop[19] == 0x1C
    pl_bmf.apply_id04(d4_stop)
    assert pl_bmf.hw_moving == 0
    pl_bmf.b6_flags = 0x00
    pl_bmf.feed(c_bmf)
    d4_idle = c_bmf.build_id04()
    assert d4_idle[9] == 0x81

    # Barrel-MF ignored 0x1D must not emit AF_STALL_ABORT and must settle to 0x81
    c_ign = CoreHarness(lib)
    pl_ign = VarifocalPlant(f_mm=150.0, dist_m=1.96, is_mf=1, ignore_1d=True)
    complete_mock_handshake(
        c_ign, 0x807B, SONY_CAPS,
        make_id05_108b(dist_x=pl_ign.dist_x, f_opt_0p1=pl_ign.f01, f_nom_0p1=pl_ign.f01, is_mf=1),
        make_id06_39b(pl_ign.focus_se, pl_ign.ei, pl_ign.en, zoom_norm=pl_ign.zn),
    )
    pl_ign.set_zoom(100.0)
    for _ in range(20):
        pl_ign.feed(c_ign)
        d4_i = c_ign.build_id04()
    ev_buf = ctypes.create_string_buffer(20)
    stalls = 0
    while lib.lens_session_pop_diag(c_ign.ctx, ev_buf):
        if struct.unpack("<HHIIII", ev_buf.raw)[0] == BB_EVT_AF_STALL_ABORT:
            stalls += 1
    assert stalls == 0 and d4_i[9] == 0x81

    # 0xC56F 120 Hz lag=4 zoom roundtrip + <=2 SE lim_far jitter keeping mech_lz == 0
    c_6f = CoreHarness(lib)
    spec_6f = (70.0, 200.0, 5672, 8106, 3200, 19600, 0.65, 1.00, 10.0)
    pl_6f = VarifocalPlant(f_mm=200.0, dist_m=2.0, dist_lag_frames=4, cam_spec=spec_6f)
    complete_mock_handshake(
        c_6f, 0xC56F, SONY_CAPS,
        make_id05_108b(dist_x=pl_6f.dist_x, f_opt_0p1=pl_6f.f01, f_nom_0p1=pl_6f.f01),
        make_id06_39b(pl_6f.focus_se, pl_6f.ei, pl_6f.en, zoom_norm=pl_6f.zn),
        tx08_ois=1, tx08_120hz=1,
    )
    for _ in range(3):
        pl_6f.feed(c_6f)
        _ = c_6f.build_id04()
    anch_6f = pl_6f.true_dist_x
    for k in range(1, 16):
        pl_6f.set_zoom(200.0 - 130.0 * (k / 15.0))
        pl_6f.feed(c_6f)
        pl_6f.apply_id04(c_6f.build_id04())
    for _ in range(28):
        pl_6f.feed(c_6f)
        pl_6f.apply_id04(c_6f.build_id04())
    assert abs(pl_6f.true_dist_x - anch_6f) <= 1

    c_jit = CoreHarness(lib)
    complete_mock_handshake(
        c_jit, 0xC56F, SONY_CAPS,
        make_id05_108b(dist_x=0xFFFF, f_opt_0p1=700, f_nom_0p1=700),
        make_id06_39b(5672, 5672, 8106, zoom_norm=0x0100),
        tx08_ois=1, tx08_120hz=1,
    )
    for far_se in (5672, 5671, 5670):
        c_jit.feed_emount_frame(
            1, 0x05,
            make_id05_108b(dist_x=0xFFFF, f_opt_0p1=700, f_nom_0p1=700)
            + b"\x06"
            + make_id06_39b(far_se, far_se, 8106, zoom_norm=0x0100),
        )
        _ = c_jit.build_id04()
        c20_j = c_jit.z_exchange(0x20, 0x00, 258)
        hot_j = c_jit.build_hotline(1)
        assert struct.unpack_from("<i", c20_j, 0x75)[0] == 0
        assert struct.unpack_from("<i", hot_j, 2)[0] == 0

    return {"mode_errs": mode_errs, "body_mf_e0_brake_ok": True, "c56f_120hz_jitter_ok": True}


def test_c56e_af_default_and_ois_toggle(lib: ctypes.CDLL) -> dict[str, Any]:
    """Verify slow-homing 0xC56E AF default gate and CMD 0xD2 OIS enablement."""
    core = CoreHarness(lib)
    complete_mock_handshake(
        core, 0xC56E, SIGMA_CAPS,
        make_id05_108b(dist_x=486, f_opt_0p1=1500, f_nom_0p1=1500, is_mf=0),
        make_id06_39b(12000, 9093, 24052, zoom_norm=0x0100),
        tx08_ois=1, tx08_120hz=1, drain_startup_id04=False,
    )
    c20_init = core.z_exchange(0x20, 0x00, 258)
    assert (c20_init[0xAC] & 0x80) == 0, "Cold-boot AF lens unexpectedly defaulted to MF"
    id03_off = core.build_id03()
    assert struct.unpack_from("<H", id03_off, 19)[0] == 0x0033 and (id03_off[21] & 0x01) == 0

    core.z_exchange(0xD2, 0x00, 4, mosi=seal_z_frame(bytearray([0x55, 0x02, 0x00, 0x00])))
    id03_on = core.build_id03()
    assert (id03_on[21] & 0x01) == 0x01 and struct.unpack_from("<H", id03_on, 19)[0] != 0x0033
    return {"c56e_af_default_ok": True, "ois_d2_toggle_ok": True}


def main() -> int:
    lib = load_core_library()
    res = {
        "status": "PASS",
        "lib": str(LIB_PATH.name),
        "catalog_and_optics": test_catalog_and_optics(lib),
        "parfocal_and_af": test_fulltime_parfocal_and_body_mf_e0(lib),
        "c56e_and_ois": test_c56e_af_default_and_ois_toggle(lib),
    }
    print(json.dumps(res, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
