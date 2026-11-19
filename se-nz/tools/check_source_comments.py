#!/usr/bin/env python3
"""Enforce the source-comment vocabulary contract."""
import argparse
import re
from pathlib import Path

TOKENS = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|/\*.*?\*/|//[^\n]*', re.S)
FORBIDDEN = re.compile(
    r"0[xX][0-9a-fA-F]+|\b[0-9A-Fa-f]{2}(?:\s+[0-9A-Fa-f]{2})+\b|"
    r"\b(?:sony|nikon|sigma|tamron|samyang|zeiss|voigtlander|viltrox|tokina|laowa|"
    r"7artisans|ttartisan|meike|songraw|thypoch|sirui|megadap|techart|boryoza|"
    r"etz\d*|tze\d*|xhsc|cmsis|ddl|sdk|z8|z9|z6|z7|z5|z30|zf|zfc|z50|"
    r"hc32f460\w*|apm32\w*|atsamd\w*|cortex)\b", re.I
)


def violations(text):
    return [(text.count("\n", 0, match.start()) + 1, match.group())
            for match in TOKENS.finditer(text)
            if match.group().startswith(("//", "/*")) and FORBIDDEN.search(match.group())]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        assert violations("/* 0x42 */")
        assert violations("// FF 00")
        assert violations("/* Cortex-M4 */")
        assert not violations('const char *str = "Sony /* 0x42 */"; /* frame count */')
    root = Path(__file__).resolve().parents[1]
    count = 0
    hits = []
    for folder in ("src", "include", "ld"):
        for path in (root / folder).rglob("*"):
            if path.suffix not in (".c", ".h", ".S", ".s", ".ld"):
                continue
            count += 1
            hits.extend((str(path.relative_to(root)), line, text)
                        for line, text in violations(path.read_text()))
    for hit in hits:
        print(hit)
    print(f"comment_scan files={count} violations={len(hits)}")
    return int(bool(hits))


if __name__ == "__main__":
    raise SystemExit(main())
