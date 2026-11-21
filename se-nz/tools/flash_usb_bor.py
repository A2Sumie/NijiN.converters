#!/usr/bin/env python3
"""Contiguous macOS/POSIX USB MSC Flasher & Verifier for BORYOZA `BOR` Bootloader.

Handles BORYOZA Bootloader FAT12 root volume label `"BOR     \\x00\\x00 "` (@0xA20), which
macOS FSKit (`msdos`) mounts with five trailing spaces as `/Volumes/BOR     `, and writes `.PUB`
images in a single contiguous extent compatible with BL writer `0x05070..0x05158`.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
FW_ROOT = HERE.parent
DEFAULT_PUB = (
    HERE / "BORYOZA_V224.PUB"
    if (HERE / "BORYOZA_V224.PUB").is_file()
    else FW_ROOT / "build/BORYOZA_V224.PUB"
)
DEFAULT_VOLUMES_ROOT = Path("/Volumes")
BOR_VOLUME_STEM = "BOR"
PUB_MAGIC = b"PRODUCTVER:1.001"
README_VER_PUB_OFFSET = 0x1010

# macOS <sys/fcntl.h> constants
F_PREALLOCATE = 42
F_NOCACHE = 48
F_FULLFSYNC = 51
F_ALLOCATECONTIG = 0x00000002
F_ALLOCATEALL = 0x00000004
F_PEOFPOSMODE = 3

OPTIONAL_FCNTL_ERRNOS = {
    errno.ENOTSUP,
    errno.EOPNOTSUPP,
    errno.EINVAL,
    errno.ENOTTY,
}


class FStore(ctypes.Structure):
    _fields_ = [
        ("fst_flags", ctypes.c_uint32),
        ("fst_posmode", ctypes.c_int),
        ("fst_offset", ctypes.c_int64),
        ("fst_length", ctypes.c_int64),
        ("fst_bytesalloc", ctypes.c_int64),
    ]


def expected_version_from_pub(pub_bytes: bytes) -> tuple[str, str]:
    if (
        len(pub_bytes) < README_VER_PUB_OFFSET + 4
        or (len(pub_bytes) % 512) != 0
        or pub_bytes[:16] != PUB_MAGIC
    ):
        raise ValueError("Invalid .PUB image header or sector alignment")
    minor, major, month, year = pub_bytes[README_VER_PUB_OFFSET : README_VER_PUB_OFFSET + 4]
    return f"V{major:d}.{minor:02d}", f"20{year:02d}-{month:02d}"


def read_bor_readme(mount_dir: Path) -> str:
    readme = mount_dir / "README.TXT"
    if not readme.exists():
        return ""
    raw = readme.read_bytes().split(b"\x00", 1)[0]
    return raw.decode("ascii", errors="replace").strip()


def resolve_bor_mount(
    mount_arg: Path | None = None,
    volumes_root: Path = DEFAULT_VOLUMES_ROOT,
) -> Path | None:
    if mount_arg is not None and (mount_arg / "README.TXT").exists():
        return mount_arg

    if mount_arg is not None:
        search_root = mount_arg.parent
        target_stem = mount_arg.name.rstrip(" \x00").upper()
    else:
        search_root = volumes_root
        target_stem = BOR_VOLUME_STEM

    if not search_root.is_dir():
        return None

    candidates: list[Path] = []
    for entry in sorted(search_root.iterdir(), key=lambda p: p.name):
        if not entry.is_dir():
            continue
        if entry.name.rstrip(" \x00").upper() != target_stem:
            continue
        if (entry / "README.TXT").exists():
            if "BORYOZA" in read_bor_readme(entry).upper():
                return entry
            candidates.append(entry)

    return candidates[0] if candidates else None


def wait_for_bor_mount(
    mount_arg: Path | None,
    timeout_s: float,
    volumes_root: Path = DEFAULT_VOLUMES_ROOT,
) -> Path | None:
    deadline = time.monotonic() + timeout_s
    while True:
        resolved = resolve_bor_mount(mount_arg, volumes_root)
        if resolved is not None:
            return resolved
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.05)


def preallocate_contiguous_darwin(fd: int, length: int) -> int:
    if sys.platform != "darwin":
        return 0
    store = FStore(
        fst_flags=F_ALLOCATECONTIG | F_ALLOCATEALL,
        fst_posmode=F_PEOFPOSMODE,
        fst_offset=0,
        fst_length=length,
        fst_bytesalloc=0,
    )
    try:
        raw = fcntl.fcntl(fd, F_PREALLOCATE, bytes(store))
    except OSError as exc:
        if exc.errno in OPTIONAL_FCNTL_ERRNOS:
            return 0
        raise
    return int(FStore.from_buffer_copy(raw).fst_bytesalloc)


def set_nocache_darwin(fd: int) -> None:
    if sys.platform != "darwin":
        return
    try:
        fcntl.fcntl(fd, F_NOCACHE, 1)
    except OSError as exc:
        if exc.errno not in OPTIONAL_FCNTL_ERRNOS:
            raise


def sync_fd(fd: int) -> None:
    if sys.platform == "darwin":
        try:
            fcntl.fcntl(fd, F_FULLFSYNC)
            return
        except OSError as exc:
            if exc.errno not in OPTIONAL_FCNTL_ERRNOS:
                raise
    os.fsync(fd)


def is_darwin_volumes_mount(mount_dir: Path) -> bool:
    return sys.platform == "darwin" and mount_dir.parent == DEFAULT_VOLUMES_ROOT


def eject_mount(mount_dir: Path) -> bool:
    proc = subprocess.run(
        ["diskutil", "eject", str(mount_dir)],
        capture_output=True,
        text=True,
        check=False,
    )
    return proc.returncode == 0


def flash_pub_to_bor(
    pub_path: Path,
    mount_dir: Path,
    eject_after: bool = True,
) -> dict[str, object]:
    pub_bytes = pub_path.read_bytes()
    expected_ver, expected_date = expected_version_from_pub(pub_bytes)

    before_readme = read_bor_readme(mount_dir)
    user_files = [
        p.name
        for p in mount_dir.iterdir()
        if p.name.upper() != "README.TXT" and not p.name.startswith(".")
    ]
    if user_files:
        raise RuntimeError(
            f"{mount_dir!r} has existing files {user_files}; unplug and replug USB cap first."
        )

    dst_path = mount_dir / pub_path.name
    fd = os.open(str(dst_path), os.O_CREAT | os.O_WRONLY | os.O_TRUNC, 0o644)
    alloc_bytes = 0
    try:
        set_nocache_darwin(fd)
        alloc_bytes = preallocate_contiguous_darwin(fd, len(pub_bytes))
        total_written = 0
        mv = memoryview(pub_bytes)
        while total_written < len(pub_bytes):
            chunk = os.write(fd, mv[total_written:])
            if chunk <= 0:
                raise OSError(f"Short write at {total_written}/{len(pub_bytes)}")
            total_written += chunk
        sync_fd(fd)
    finally:
        os.close(fd)

    ejected = False
    if eject_after and is_darwin_volumes_mount(mount_dir):
        ejected = eject_mount(mount_dir)

    return {
        "status": "FLASHED",
        "mount_dir": str(mount_dir),
        "mount_dir_repr": repr(str(mount_dir)),
        "pub": str(pub_path),
        "pub_bytes": len(pub_bytes),
        "pub_sha256": hashlib.sha256(pub_bytes).hexdigest(),
        "expected_version": expected_ver,
        "expected_date": expected_date,
        "contiguous_prealloc_bytes": alloc_bytes,
        "ejected": ejected,
        "before_readme": before_readme,
    }


def run_selftest() -> int:
    import tempfile

    pub_bytes = DEFAULT_PUB.read_bytes()
    expected_ver, expected_date = expected_version_from_pub(pub_bytes)
    assert expected_ver == "V2.24", expected_ver
    assert expected_date == "2026-11", expected_date

    with tempfile.TemporaryDirectory() as tmp:
        vroot = Path(tmp)
        mdir = vroot / "BOR     "
        mdir.mkdir()
        (mdir / "README.TXT").write_bytes(
            b"ProductName: BORYOZA SONY E TO NIKON Z ADAPTER\r\n"
            b"Software Version: V2.09\r\n\x00"
        )
        assert resolve_bor_mount(None, vroot) == mdir
        assert resolve_bor_mount(vroot / "BOR", vroot) == mdir

        res = flash_pub_to_bor(DEFAULT_PUB, mdir, eject_after=False)
        written = (mdir / DEFAULT_PUB.name).read_bytes()
        assert written == pub_bytes
        assert not (mdir / f"._{DEFAULT_PUB.name}").exists()
        print(
            json.dumps(
                {
                    "selftest": "PASS",
                    "resolved_mount_repr": res["mount_dir_repr"],
                    "expected_version": expected_ver,
                    "bytes": res["pub_bytes"],
                    "prealloc": res["contiguous_prealloc_bytes"],
                }
            )
        )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Contiguous USB BOR flasher & verifier for BORYOZA .PUB firmware"
    )
    ap.add_argument("--pub", type=Path, default=DEFAULT_PUB)
    ap.add_argument(
        "--mount",
        type=Path,
        default=None,
        help="Explicit BOR mount path (default: auto-discover /Volumes/BOR* including trailing spaces)",
    )
    ap.add_argument("--wait-seconds", type=float, default=30.0)
    ap.add_argument("--no-eject", action="store_true")
    ap.add_argument("--verify-only", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()

    mount_dir = wait_for_bor_mount(args.mount, args.wait_seconds)
    if mount_dir is None:
        target_desc = str(args.mount) if args.mount is not None else "/Volumes/BOR*"
        print(
            json.dumps(
                {
                    "status": "ERROR",
                    "error": f"{target_desc} not mounted within {args.wait_seconds}s",
                }
            )
        )
        return 1

    if args.verify_only:
        expected_ver, expected_date = expected_version_from_pub(args.pub.read_bytes())
        readme_txt = read_bor_readme(mount_dir)
        ok = expected_ver in readme_txt
        print(
            json.dumps(
                {
                    "status": "PASS" if ok else "FAIL",
                    "mount_dir_repr": repr(str(mount_dir)),
                    "expected_version": expected_ver,
                    "expected_date": expected_date,
                    "readme": readme_txt,
                },
                ensure_ascii=False,
            )
        )
        return 0 if ok else 1

    res = flash_pub_to_bor(args.pub, mount_dir, eject_after=not args.no_eject)
    print(json.dumps(res, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
