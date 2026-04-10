from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable, Protocol

from sim_compare.models import InputControlCommand
from sim_compare.utils import clamp


class ControlSource(Protocol):
    def next(
        self, sim_time_s: float, step: int, milliseconds: int = 0
    ) -> tuple[InputControlCommand, bool]:
        ...

    def close(self) -> None:
        ...


class SeriesControlSource:
    def __init__(self, csv_path: Path, hold_last: bool):
        self.rows = list(self._load(csv_path))
        if not self.rows:
            raise ValueError(f"No control rows found in {csv_path}")
        self.hold_last = hold_last
        self.index = 0
        self.timed = self.rows[0][0] is not None

    def _load(
        self, csv_path: Path
    ) -> Iterable[tuple[float | None, InputControlCommand]]:
        with csv_path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            required = {"throttle", "steer"}
            if reader.fieldnames is None or not required.issubset(
                set(reader.fieldnames)
            ):
                raise ValueError(
                    "Control CSV must include at least throttle and steer columns "
                    "(optional: brake, hand_brake, reverse, time_s)"
                )
            timed = "time_s" in reader.fieldnames
            for row in reader:
                time_s = float(row["time_s"]) if timed and row["time_s"] else None
                yield (
                    time_s,
                    InputControlCommand(
                        throttle=clamp(
                            float(row.get("throttle", 0.0) or 0.0), 0.0, 1.0
                        ),
                        brake=clamp(float(row.get("brake", 0.0) or 0.0), 0.0, 1.0),
                        steer=clamp(float(row.get("steer", 0.0) or 0.0), -1.0, 1.0),
                        hand_brake=str(row.get("hand_brake", "")).lower()
                        in {"1", "true", "yes"},
                        reverse=str(row.get("reverse", "")).lower()
                        in {"1", "true", "yes"},
                    ),
                )

    def next(
        self, sim_time_s: float, step: int, milliseconds: int = 0
    ) -> tuple[InputControlCommand, bool]:
        del milliseconds
        if self.timed:
            while self.index + 1 < len(self.rows):
                next_time, _ = self.rows[self.index + 1]
                assert next_time is not None
                if next_time <= sim_time_s:
                    self.index += 1
                else:
                    break
            if not self.hold_last and self.index == len(self.rows) - 1:
                last_time, _ = self.rows[self.index]
                assert last_time is not None
                if sim_time_s > last_time:
                    return self.rows[self.index][1], True
            return self.rows[self.index][1], False

        if step >= len(self.rows):
            if self.hold_last:
                return self.rows[-1][1], False
            return self.rows[-1][1], True
        return self.rows[step][1], False

    def close(self) -> None:
        return None


class KeyboardControlSource:
    def __init__(self, pygame_module):
        self.pg = pygame_module
        self.control = InputControlCommand()
        self.steer_cache = 0.0

    def next(
        self, sim_time_s: float, step: int, milliseconds: int = 0
    ) -> tuple[InputControlCommand, bool]:
        del sim_time_s, step
        for event in self.pg.event.get():
            if event.type == self.pg.QUIT:
                return self.control, True
            if event.type == self.pg.KEYUP:
                if event.key == self.pg.K_ESCAPE:
                    return self.control, True
                if event.key == self.pg.K_q:
                    self.control.reverse = not self.control.reverse

        keys = self.pg.key.get_pressed()
        if keys[self.pg.K_UP] or keys[self.pg.K_w]:
            self.control.throttle = min(self.control.throttle + 0.1, 1.0)
        else:
            self.control.throttle = 0.0

        if keys[self.pg.K_DOWN] or keys[self.pg.K_s]:
            self.control.brake = min(self.control.brake + 0.2, 1.0)
        else:
            self.control.brake = 0.0

        steer_increment = 5e-4 * milliseconds
        if keys[self.pg.K_LEFT] or keys[self.pg.K_a]:
            if self.steer_cache > 0.0:
                self.steer_cache = 0.0
            else:
                self.steer_cache -= steer_increment
        elif keys[self.pg.K_RIGHT] or keys[self.pg.K_d]:
            if self.steer_cache < 0.0:
                self.steer_cache = 0.0
            else:
                self.steer_cache += steer_increment
        else:
            self.steer_cache = 0.0

        self.steer_cache = clamp(self.steer_cache, -0.7, 0.7)
        self.control.steer = round(self.steer_cache, 1)
        self.control.hand_brake = bool(keys[self.pg.K_SPACE])
        return (
            InputControlCommand(
                throttle=self.control.throttle,
                brake=self.control.brake,
                steer=self.control.steer,
                hand_brake=self.control.hand_brake,
                reverse=self.control.reverse,
            ),
            False,
        )

    def close(self) -> None:
        return None
