from __future__ import annotations

import csv
from pathlib import Path

from sim_compare.models import InitialPose


GENERAL_PROFILE = "general"
LONGITUDINAL_ONLY_PROFILE = "longitudinal_only"
LATERAL_ONLY_PROFILE = "lateral_only"
COUPLED_MANEUVER_PROFILE = "coupled_maneuver"
SUPPORTED_EXPERIMENT_PROFILES = {
    GENERAL_PROFILE,
    LONGITUDINAL_ONLY_PROFILE,
    LATERAL_ONLY_PROFILE,
    COUPLED_MANEUVER_PROFILE,
}


def normalize_experiment_profile(profile: str) -> str:
    key = profile.strip().lower()
    aliases = {
        "general": GENERAL_PROFILE,
        "longitudinal": LONGITUDINAL_ONLY_PROFILE,
        "longitudinal_only": LONGITUDINAL_ONLY_PROFILE,
        "lateral": LATERAL_ONLY_PROFILE,
        "lateral_only": LATERAL_ONLY_PROFILE,
        "coupled": COUPLED_MANEUVER_PROFILE,
        "coupled_maneuver": COUPLED_MANEUVER_PROFILE,
        "coupled_maneuvers": COUPLED_MANEUVER_PROFILE,
    }
    normalized = aliases.get(key, key)
    if normalized not in SUPPORTED_EXPERIMENT_PROFILES:
        raise ValueError(f"Unsupported experiment profile: {profile}")
    return normalized


def _load_control_rows(csv_path: Path) -> list[dict[str, float | bool]]:
    rows: list[dict[str, float | bool]] = []
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(
                {
                    "throttle": float(row.get("throttle", 0.0) or 0.0),
                    "brake": float(row.get("brake", 0.0) or 0.0),
                    "steer": float(row.get("steer", 0.0) or 0.0),
                    "hand_brake": str(row.get("hand_brake", "")).lower()
                    in {"1", "true", "yes"},
                    "reverse": str(row.get("reverse", "")).lower()
                    in {"1", "true", "yes"},
                }
            )
    return rows


def validate_experiment_profile(
    profile: str,
    initial_pose: InitialPose,
    input_mode: str,
    control_csv: Path | None,
) -> list[str]:
    normalized = normalize_experiment_profile(profile)
    messages: list[str] = []

    if normalized == GENERAL_PROFILE:
        return messages

    if normalized == LATERAL_ONLY_PROFILE and initial_pose.speed <= 0.0:
        messages.append(
            "lateral_only profile requires a positive --init-speed so steering-only tests start from a controlled moving state"
        )

    if input_mode != "series" or control_csv is None:
        if normalized in {LONGITUDINAL_ONLY_PROFILE, LATERAL_ONLY_PROFILE}:
            messages.append(
                f"{normalized} profile cannot fully validate live keyboard input; use --input-mode series for strict reproducibility"
            )
        return messages

    rows = _load_control_rows(control_csv)
    if normalized == LONGITUDINAL_ONLY_PROFILE:
        if any(abs(float(row["steer"])) > 1e-6 for row in rows):
            messages.append(
                "longitudinal_only profile expects every control row to keep steer=0"
            )
    elif normalized == LATERAL_ONLY_PROFILE:
        if any(abs(float(row["throttle"])) > 1e-6 for row in rows):
            messages.append(
                "lateral_only profile expects every control row to keep throttle=0"
            )
        if any(abs(float(row["brake"])) > 1e-6 for row in rows):
            messages.append(
                "lateral_only profile expects every control row to keep brake=0"
            )
        if any(bool(row["hand_brake"]) for row in rows):
            messages.append(
                "lateral_only profile expects every control row to keep hand_brake=0"
            )
        if any(bool(row["reverse"]) for row in rows):
            messages.append(
                "lateral_only profile expects every control row to keep reverse=0"
            )
    return messages
