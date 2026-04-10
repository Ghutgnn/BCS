from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class MapPaths:
    map_name: str
    maps_dir: Path
    xosc_path: Path
    xodr_path: Path


def resolve_map_paths(project_root: Path, map_name: str) -> MapPaths:
    maps_dir = (project_root / "maps").resolve()
    xosc_path = maps_dir / f"{map_name}.xosc"
    xodr_path = maps_dir / f"{map_name}.xodr"
    if not xosc_path.exists():
        raise FileNotFoundError(f"esmini map not found: {xosc_path}")
    if not xodr_path.exists():
        raise FileNotFoundError(f"CARLA map not found: {xodr_path}")
    return MapPaths(
        map_name=map_name,
        maps_dir=maps_dir,
        xosc_path=xosc_path,
        xodr_path=xodr_path,
    )


def build_esmini_search_paths(
    esmini_home: Path,
    map_paths: MapPaths,
    extra_paths: list[Path],
) -> list[Path]:
    candidates = [
        map_paths.maps_dir,
        map_paths.xosc_path.parent,
        map_paths.xodr_path.parent,
        esmini_home / "resources",
        esmini_home / "resources" / "xosc",
        esmini_home / "resources" / "xodr",
        esmini_home / "resources" / "models",
    ]
    result: list[Path] = []
    seen: set[Path] = set()
    for candidate in [*extra_paths, *candidates]:
        resolved = candidate.resolve()
        if resolved.exists() and resolved not in seen:
            result.append(resolved)
            seen.add(resolved)
    return result
