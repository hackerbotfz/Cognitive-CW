#!/usr/bin/env python3
"""Gzip a YOLO .pt checkpoint for storage in git (often 20–40% smaller)."""

import argparse
import gzip
import os
import shutil
import sys


def main():
    parser = argparse.ArgumentParser(description="Compress a YOLO weights file to .pt.gz")
    parser.add_argument(
        "input",
        nargs="?",
        default=os.path.join(
            os.path.dirname(__file__),
            "..",
            "src",
            "ntu_robotsim",
            "models",
            "best.pt",
        ),
        help="Path to best.pt (default: src/ntu_robotsim/models/best.pt)",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output path (default: input path + .gz)",
    )
    args = parser.parse_args()

    src = os.path.abspath(args.input)
    if not os.path.isfile(src):
        print(f"Not found: {src}", file=sys.stderr)
        sys.exit(1)

    dst = os.path.abspath(args.output or src + ".gz")
    before = os.path.getsize(src)

    with open(src, "rb") as f_in:
        with gzip.open(dst, "wb", compresslevel=9) as f_out:
            shutil.copyfileobj(f_in, f_out)

    after = os.path.getsize(dst)
    ratio = 100 * (1 - after / before) if before else 0
    print(f"Wrote {dst}")
    print(f"  {before / 1e6:.2f} MB -> {after / 1e6:.2f} MB ({ratio:.1f}% smaller)")


if __name__ == "__main__":
    main()
