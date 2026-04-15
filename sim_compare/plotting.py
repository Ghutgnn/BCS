from __future__ import annotations

import csv
import html
from pathlib import Path


SVG_WIDTH = 1400
SVG_HEIGHT = 960
PLOT_LEFT = 80
PLOT_TOP = 80
PLOT_RIGHT = 1120
PLOT_BOTTOM = 880
LEGEND_X = 1160
SIMULATOR_COLORS = [
    {
        "dark": (26, 115, 232),
        "light": (219, 233, 255),
        "stroke": "rgb(26,115,232)",
    },
    {
        "dark": (220, 96, 24),
        "light": (255, 230, 212),
        "stroke": "rgb(220,96,24)",
    },
]


def _speed_field_name(row: dict[str, float | str], prefix: str) -> str:
    planar_name = f"{prefix}_speed_planar"
    return planar_name if planar_name in row else f"{prefix}_speed"


def _parse_rows(csv_path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            parsed: dict[str, float | str] = {}
            for key, value in row.items():
                normalized_key = key.strip() if key is not None else None
                normalized_value = value.strip() if isinstance(value, str) else value
                if normalized_key is None or normalized_value in (None, ""):
                    continue
                try:
                    parsed[normalized_key] = float(normalized_value)
                except ValueError:
                    parsed[normalized_key] = normalized_value
            rows.append(parsed)
    return rows


def _mix_rgb(a: tuple[int, int, int], b: tuple[int, int, int], t: float) -> str:
    clamped_t = max(0.0, min(1.0, t))
    mixed = tuple(round(a[i] + (b[i] - a[i]) * clamped_t) for i in range(3))
    return f"rgb({mixed[0]},{mixed[1]},{mixed[2]})"


def _get_participant_info(
    rows: list[dict[str, float | str]],
) -> tuple[dict[str, str], dict[str, str]]:
    first = rows[0]
    reference_prefix = str(first.get("reference_csv_prefix", "carla"))
    candidate_prefix = str(first.get("candidate_csv_prefix", "esmini"))
    reference = {
        "prefix": reference_prefix,
        "label": str(first.get("reference_label", reference_prefix)),
        "simulator_id": str(first.get("reference_simulator_id", reference_prefix)),
        "backend": str(first.get("reference_backend", "native")),
    }
    candidate = {
        "prefix": candidate_prefix,
        "label": str(first.get("candidate_label", candidate_prefix)),
        "simulator_id": str(first.get("candidate_simulator_id", candidate_prefix)),
        "backend": str(first.get("candidate_backend", "native")),
    }
    return reference, candidate


def _compute_bounds(
    rows: list[dict[str, float | str]],
    prefixes: tuple[str, str],
) -> tuple[float, float, float, float]:
    xs = [row[f"{prefixes[0]}_x"] for row in rows] + [
        row[f"{prefixes[1]}_x"] for row in rows
    ]
    ys = [row[f"{prefixes[0]}_y"] for row in rows] + [
        row[f"{prefixes[1]}_y"] for row in rows
    ]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    padding_x = span_x * 0.06
    padding_y = span_y * 0.06
    return (
        min_x - padding_x,
        max_x + padding_x,
        min_y - padding_y,
        max_y + padding_y,
    )


def _make_projector(
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
):
    plot_w = PLOT_RIGHT - PLOT_LEFT
    plot_h = PLOT_BOTTOM - PLOT_TOP
    scale_x = plot_w / max(max_x - min_x, 1e-9)
    scale_y = plot_h / max(max_y - min_y, 1e-9)
    scale = min(scale_x, scale_y)
    x_offset = PLOT_LEFT + (plot_w - (max_x - min_x) * scale) * 0.5
    y_offset = PLOT_TOP + (plot_h - (max_y - min_y) * scale) * 0.5

    def project(x: float, y: float) -> tuple[float, float]:
        sx = x_offset + (x - min_x) * scale
        sy = PLOT_BOTTOM - (y_offset - PLOT_TOP) - (y - min_y) * scale
        return sx, sy

    return project


def _build_grid(
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    project,
) -> str:
    parts = [
        f'<rect x="{PLOT_LEFT}" y="{PLOT_TOP}" width="{PLOT_RIGHT - PLOT_LEFT}" '
        f'height="{PLOT_BOTTOM - PLOT_TOP}" fill="white" stroke="#d0d7de" stroke-width="1.5"/>'
    ]
    for idx in range(11):
        tx = min_x + (max_x - min_x) * idx / 10.0
        ty = min_y + (max_y - min_y) * idx / 10.0
        x1, y_bottom = project(tx, min_y)
        _, y_top = project(tx, max_y)
        x_left, y1 = project(min_x, ty)
        x_right, _ = project(max_x, ty)
        parts.append(
            f'<line x1="{x1:.2f}" y1="{y_top:.2f}" x2="{x1:.2f}" y2="{y_bottom:.2f}" '
            f'stroke="#eef2f6" stroke-width="1"/>'
        )
        parts.append(
            f'<line x1="{x_left:.2f}" y1="{y1:.2f}" x2="{x_right:.2f}" y2="{y1:.2f}" '
            f'stroke="#eef2f6" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{x1:.2f}" y="{PLOT_BOTTOM + 24}" text-anchor="middle" '
            f'font-size="12" fill="#475467">{tx:.1f}</text>'
        )
        parts.append(
            f'<text x="{PLOT_LEFT - 12}" y="{y1 + 4:.2f}" text-anchor="end" '
            f'font-size="12" fill="#475467">{ty:.1f}</text>'
        )
    parts.append(
        f'<text x="{(PLOT_LEFT + PLOT_RIGHT) / 2:.2f}" y="{SVG_HEIGHT - 20}" '
        f'text-anchor="middle" font-size="15" fill="#111827">x [m]</text>'
    )
    parts.append(
        f'<text x="24" y="{(PLOT_TOP + PLOT_BOTTOM) / 2:.2f}" transform="rotate(-90 24,{(PLOT_TOP + PLOT_BOTTOM) / 2:.2f})" '
        f'text-anchor="middle" font-size="15" fill="#111827">y [m]</text>'
    )
    return "\n".join(parts)


def _build_segments(
    rows: list[dict[str, float | str]],
    prefix: str,
    light_rgb: tuple[int, int, int],
    base_rgb: tuple[int, int, int],
    project,
    max_speed: float,
) -> str:
    if len(rows) < 2:
        return ""
    parts: list[str] = []
    for idx in range(len(rows) - 1):
        row_a = rows[idx]
        row_b = rows[idx + 1]
        x1, y1 = project(row_a[f"{prefix}_x"], row_a[f"{prefix}_y"])
        x2, y2 = project(row_b[f"{prefix}_x"], row_b[f"{prefix}_y"])
        speed_field_a = _speed_field_name(row_a, prefix)
        speed_field_b = _speed_field_name(row_b, prefix)
        avg_speed = 0.5 * (row_a[speed_field_a] + row_b[speed_field_b])
        speed_t = 0.0 if max_speed <= 1e-9 else avg_speed / max_speed
        color = _mix_rgb(light_rgb, base_rgb, speed_t)
        parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{color}" stroke-width="5" stroke-linecap="round" opacity="0.96"/>'
        )
    return "\n".join(parts)


def _build_markers(
    rows: list[dict[str, float | str]],
    prefix: str,
    color: str,
    label: str,
    project,
) -> str:
    start = rows[0]
    end = rows[-1]
    sx, sy = project(start[f"{prefix}_x"], start[f"{prefix}_y"])
    ex, ey = project(end[f"{prefix}_x"], end[f"{prefix}_y"])
    return "\n".join(
        [
            f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="8" fill="white" stroke="{color}" stroke-width="3"/>',
            f'<text x="{sx + 12:.2f}" y="{sy - 10:.2f}" font-size="13" fill="{color}">{html.escape(label)} start</text>',
            f'<rect x="{ex - 7:.2f}" y="{ey - 7:.2f}" width="14" height="14" fill="white" stroke="{color}" stroke-width="3" transform="rotate(45 {ex:.2f} {ey:.2f})"/>',
            f'<text x="{ex + 12:.2f}" y="{ey + 18:.2f}" font-size="13" fill="{color}">{html.escape(label)} end</text>',
        ]
    )


def _build_legend(
    rows: list[dict[str, float | str]],
    title: str,
    map_name: str,
    reference: dict[str, str],
    candidate: dict[str, str],
    reference_color: str,
    candidate_color: str,
) -> str:
    mean_diff = sum(row["diff_pos_2d"] for row in rows) / len(rows)
    max_diff = max(row["diff_pos_2d"] for row in rows)
    final_diff = rows[-1]["diff_pos_2d"]
    max_speed = max(
        max(
            row[_speed_field_name(row, reference["prefix"])],
            row[_speed_field_name(row, candidate["prefix"])],
        )
        for row in rows
    )
    parts = [
        f'<text x="{LEGEND_X}" y="92" font-size="24" font-weight="700" fill="#111827">{html.escape(title)}</text>',
        f'<text x="{LEGEND_X}" y="122" font-size="15" fill="#475467">map: {html.escape(map_name)}</text>',
        f'<text x="{LEGEND_X}" y="146" font-size="15" fill="#475467">reference: {html.escape(reference["label"])} ({html.escape(reference["backend"])})</text>',
        f'<text x="{LEGEND_X}" y="168" font-size="15" fill="#475467">candidate: {html.escape(candidate["label"])} ({html.escape(candidate["backend"])})</text>',
        f'<text x="{LEGEND_X}" y="212" font-size="16" font-weight="700" fill="#111827">Trajectory Overlay</text>',
        f'<line x1="{LEGEND_X}" y1="242" x2="{LEGEND_X + 70}" y2="242" stroke="{reference_color}" stroke-width="6" stroke-linecap="round"/>',
        f'<text x="{LEGEND_X + 88}" y="247" font-size="14" fill="#111827">{html.escape(reference["label"])} trajectory</text>',
        f'<line x1="{LEGEND_X}" y1="272" x2="{LEGEND_X + 70}" y2="272" stroke="{candidate_color}" stroke-width="6" stroke-linecap="round"/>',
        f'<text x="{LEGEND_X + 88}" y="277" font-size="14" fill="#111827">{html.escape(candidate["label"])} trajectory</text>',
        f'<text x="{LEGEND_X}" y="330" font-size="16" font-weight="700" fill="#111827">Speed Encoding</text>',
        f'<text x="{LEGEND_X}" y="356" font-size="14" fill="#475467">Lighter = slower, darker = faster</text>',
        f'<text x="{LEGEND_X}" y="378" font-size="14" fill="#475467">max speed in log: {max_speed:.2f} m/s</text>',
        f'<text x="{LEGEND_X}" y="440" font-size="16" font-weight="700" fill="#111827">Distance Summary</text>',
        f'<text x="{LEGEND_X}" y="468" font-size="14" fill="#475467">mean 2D diff: {mean_diff:.3f} m</text>',
        f'<text x="{LEGEND_X}" y="492" font-size="14" fill="#475467">max 2D diff: {max_diff:.3f} m</text>',
        f'<text x="{LEGEND_X}" y="516" font-size="14" fill="#475467">final 2D diff: {final_diff:.3f} m</text>',
    ]
    return "\n".join(parts)


def render_svg_from_csv(
    csv_path: Path,
    output_path: Path | None = None,
    title: str | None = None,
) -> Path:
    rows = _parse_rows(csv_path)
    if not rows:
        raise ValueError(f"No rows found in {csv_path}")

    if output_path is None:
        output_path = csv_path.with_suffix(".svg")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    reference, candidate = _get_participant_info(rows)
    map_name = csv_path.stem.replace("comparison_", "")
    plot_title = title or f"{reference['label']} vs {candidate['label']} Trajectory"
    min_x, max_x, min_y, max_y = _compute_bounds(
        rows,
        (reference["prefix"], candidate["prefix"]),
    )
    project = _make_projector(min_x, max_x, min_y, max_y)
    max_speed = max(
        max(
            row[_speed_field_name(row, reference["prefix"])],
            row[_speed_field_name(row, candidate["prefix"])],
        )
        for row in rows
    )
    reference_colors = SIMULATOR_COLORS[0]
    candidate_colors = SIMULATOR_COLORS[1]

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{SVG_WIDTH}" height="{SVG_HEIGHT}" viewBox="0 0 {SVG_WIDTH} {SVG_HEIGHT}">
<rect width="100%" height="100%" fill="#f8fafc"/>
{_build_grid(min_x, max_x, min_y, max_y, project)}
{_build_segments(rows, reference["prefix"], reference_colors["light"], reference_colors["dark"], project, max_speed)}
{_build_segments(rows, candidate["prefix"], candidate_colors["light"], candidate_colors["dark"], project, max_speed)}
{_build_markers(rows, reference["prefix"], reference_colors["stroke"], reference["label"], project)}
{_build_markers(rows, candidate["prefix"], candidate_colors["stroke"], candidate["label"], project)}
{_build_legend(rows, plot_title, map_name, reference, candidate, reference_colors["stroke"], candidate_colors["stroke"])}
</svg>
'''
    output_path.write_text(svg, encoding="utf-8")
    return output_path
