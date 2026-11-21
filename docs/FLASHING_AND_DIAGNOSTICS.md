# Building, Flashing, and Field Diagnostics (`se-nz`)

## 1. Local Toolchain & Build

Install standard open-source tools:

- **macOS (Homebrew)**: `brew install arm-none-eabi-gcc make python` (with Xcode Command Line Tools providing `clang`)
- **Linux (Debian / Ubuntu)**: `sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi make clang python3`

Build and test from the repository root:

```sh
make
make test
```

Outputs under `se-nz/build/`:

- `se-nz/build/BORYOZA_V200.elf` — Linked Cortex-M4F ELF with debug symbols and `.map`
- `se-nz/build/BORYOZA_V200.bin` — Raw flashable APP image (`0x10000`-based)
- `se-nz/build/BORYOZA_V200.PUB` & `se-nz/build/BORYOZA_V224.PUB` — `512 B`-padded USB MSC upgrade package (`PRODUCTVER:1.001` container)
- `se-nz/build/pack_manifest.json` — Exact sector count, `program_end_hex`, VFP/closure audit counts, and SHA-256 digests
- `se-nz/build/libboryoza_core.dylib` (macOS) / `.so` (Linux) — Host ASan+UBSan shared core library

---

## 2. Flashing `.PUB` via Stock USB MSC Bootloader

The stock bootloader exposes a small FAT12 virtual disk with root volume label `"BOR     "` when connected over USB in upgrade mode. Its writer (`0x05070..0x05158`) expects the `.PUB` file to occupy contiguous clusters in the root directory and leaves `[0x00000..0x0FFFF]` (bootloader + UID block at `0x04200`) intact.

### Automated Contiguous Flasher (macOS / POSIX)

```sh
python3 se-nz/tools/flash_usb_bor.py --pub se-nz/build/BORYOZA_V224.PUB
```

Useful options:

- `--mount /path/to/BOR` — Explicitly specify mount directory (auto-discovers `/Volumes/BOR*` on macOS, including trailing spaces)
- `--no-verify` — Skip post-write flush & `README.TXT` wait step
- `--selftest` — Run self-check against a temporary FAT12 image (`0` hardware needed)

### Expected Version Token After Flash

- USB `README.TXT`: `LF 2.24` (`0x18 0x02 0x0B 0x1A` at APP offset `0x1000`)
- Nikon Z menu / EXIF firmware string (`CMD 0x10` bytes `[3..6]`): `00 00 24 02`

---

## 3. Extracting & Decoding Blackbox and Journal Logs

The firmware writes field diagnostics cleanly into the top `32 KiB` (`64` × `512 B` sectors) of internal Flash, isolated from APP code and the FAT12 window:

| LBA Range (on BOR disk) | Flash Offset | Size | Contents |
|---|---|---|---|
| `842 .. 857` | `0x78000 .. 0x79FFF` | `8 KiB` (`64 × 128 B`) | Active `Ping/Pong` Journal `JNL_PONG` (`latched_e_inf/near`, focal/aperture state, `linear_mf_code`, `vr_last_mode`, `FX/DX` LRU) |
| `858 .. 873` | `0x7A000 .. 0x7BFFF` | `8 KiB` (`256 × 32 B`) | `CRC32`-protected Blackbox ring (`BOOT_OK`, `HS_IDENTITY`, `HS_CAPS`, `Z_FIRST_WIN`, `E_LINK_UP`, `PERF_OPTICS_SNAPSHOT`, `SESSION_SUMMARY`, `V2_*_SUMMARY`, error/abort events) |
| `874 .. 905` | `0x7C000 .. 0x7FFFF` | `16 KiB` (`2 × 8 KiB`) | Active `JNL_A` (`0x7C000..0x7DFFF`) + read-only legacy `JNL_B` (`0x7E000..0x7FFFF`) |

### Extract Live USB Telemetry

```sh
python3 se-nz/tools/dump_usb_blackbox.py --out telemetry.json
```

On macOS, reading raw `/dev/rdisk*` devices requires membership in the `operator` group (`O_RDONLY`, without running Python as `root`):

```sh
sudo dseditgroup -o edit -a "$USER" -t user operator
```

Restart the terminal session after adding `$USER` to `operator`.

### Decode Offline Disk or 512 KiB Flash Images

```sh
python3 se-nz/tools/decode_blackbox.py --image flash_or_disk.bin --mode auto --out decoded.json
```

---

## 4. Further Reference / 深入参考

- Complete Documentation Index: [`docs/README.md`](README.md)
- Stock Bootloader Handoff, `PA9` Mux & FAT12 Writer (`0x5070`) State Machine: [`BOOTLOADER_AND_USB_MSC_CONTRACT.md`](ez-reference/BOOTLOADER_AND_USB_MSC_CONTRACT.md)
- `HC32F460JEUA` Clean-Room Hardware & Partition Design: [`HC32F460_SE_NZ_DESIGN_SPEC.md`](ez-reference/HC32F460_SE_NZ_DESIGN_SPEC.md)
- V2.23 Build Fingerprint, Blackbox Schema & Full Verification Matrix: [`SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md`](ez-reference/SE_NZ_V223_VERIFICATION_AND_PARFOCAL.md)
