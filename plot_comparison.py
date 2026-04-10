#!/usr/bin/env python3
"""Generate a trajectory comparison SVG from a comparison CSV."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from sim_compare.plotting import render_svg_from_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a paired-simulator trajectory overlay SVG from a comparison CSV."
    )
    parser.add_argument("csv_path", type=Path, help="comparison CSV path")
    parser.add_argument("--out", type=Path, default=None, help="output SVG path")
    parser.add_argument("--title", default=None, help="optional plot title")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    svg_path = render_svg_from_csv(
        args.csv_path.resolve(),
        output_path=args.out.resolve() if args.out else None,
        title=args.title,
    )
    print(f"Plot saved to {svg_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
