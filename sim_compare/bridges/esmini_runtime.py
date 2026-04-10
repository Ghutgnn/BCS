from __future__ import annotations

import ctypes as ct
import math
from pathlib import Path

from sim_compare.config import EsminiOptions
from sim_compare.models import VehicleState
from sim_compare.utils import normalize_yaw_rad


class SEScenarioObjectState(ct.Structure):
    _fields_ = [
        ("id", ct.c_int),
        ("model_id", ct.c_int),
        ("ctrl_type", ct.c_int),
        ("timestamp", ct.c_double),
        ("x", ct.c_double),
        ("y", ct.c_double),
        ("z", ct.c_double),
        ("h", ct.c_double),
        ("p", ct.c_double),
        ("r", ct.c_double),
        ("roadId", ct.c_uint32),
        ("junctionId", ct.c_uint32),
        ("t", ct.c_double),
        ("laneId", ct.c_int),
        ("laneOffset", ct.c_double),
        ("s", ct.c_double),
        ("speed", ct.c_double),
        ("centerOffsetX", ct.c_double),
        ("centerOffsetY", ct.c_double),
        ("centerOffsetZ", ct.c_double),
        ("width", ct.c_double),
        ("length", ct.c_double),
        ("height", ct.c_double),
        ("objectType", ct.c_int),
        ("objectCategory", ct.c_int),
        ("wheel_angle", ct.c_double),
        ("wheel_rot", ct.c_double),
        ("visibilityMask", ct.c_int),
    ]


class SESimpleVehicleState(ct.Structure):
    _fields_ = [
        ("x", ct.c_double),
        ("y", ct.c_double),
        ("z", ct.c_double),
        ("h", ct.c_double),
        ("p", ct.c_double),
        ("speed", ct.c_double),
        ("wheel_rotation", ct.c_double),
        ("wheel_angle", ct.c_double),
    ]


class EsminiRuntime:
    def __init__(
        self,
        esmini_home: Path,
        search_paths: list[Path],
        ego_index: int,
        options: EsminiOptions,
    ):
        self.search_paths = search_paths
        self.ego_index = ego_index
        self.options = options
        self.se = ct.CDLL(str(esmini_home / "bin" / "libesminiLib.so"))
        self.obj_id = -1
        self.ego_length = 5.0
        self._setup_function_signatures()

    def _setup_function_signatures(self) -> None:
        se = self.se
        se.SE_AddPath.argtypes = [ct.c_char_p]
        se.SE_AddPath.restype = ct.c_int
        se.SE_Init.argtypes = [ct.c_char_p, ct.c_int, ct.c_int, ct.c_int, ct.c_int]
        se.SE_Init.restype = ct.c_int
        se.SE_Close.argtypes = []
        se.SE_Close.restype = None
        se.SE_StepDT.argtypes = [ct.c_double]
        se.SE_StepDT.restype = ct.c_int
        se.SE_GetQuitFlag.argtypes = []
        se.SE_GetQuitFlag.restype = ct.c_int
        se.SE_GetId.argtypes = [ct.c_int]
        se.SE_GetId.restype = ct.c_int
        se.SE_GetObjectState.argtypes = [ct.c_int, ct.POINTER(SEScenarioObjectState)]
        se.SE_GetObjectState.restype = ct.c_int
        se.SE_GetObjectAcceleration.argtypes = [ct.c_int]
        se.SE_GetObjectAcceleration.restype = ct.c_double
        se.SE_SimpleVehicleCreate.argtypes = [
            ct.c_double,
            ct.c_double,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleCreate.restype = ct.c_void_p
        se.SE_SimpleVehicleDelete.argtypes = [ct.c_void_p]
        se.SE_SimpleVehicleDelete.restype = None
        se.SE_SimpleVehicleSetThrottleDisabled.argtypes = [ct.c_void_p, ct.c_bool]
        se.SE_SimpleVehicleSetThrottleDisabled.restype = None
        se.SE_SimpleVehicleSetSteeringDisabled.argtypes = [ct.c_void_p, ct.c_bool]
        se.SE_SimpleVehicleSetSteeringDisabled.restype = None
        se.SE_SimpleVehicleSetSpeed.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSetSpeed.restype = None
        se.SE_SimpleVehicleSetMaxSpeed.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSetMaxSpeed.restype = None
        se.SE_SimpleVehicleSetMaxAcceleration.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSetMaxAcceleration.restype = None
        se.SE_SimpleVehicleSetMaxDeceleration.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSetMaxDeceleration.restype = None
        se.SE_SimpleVehicleSetEngineBrakeFactor.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSetEngineBrakeFactor.restype = None
        se.SE_SimpleVehicleSteeringScale.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSteeringScale.restype = None
        se.SE_SimpleVehicleSteeringReturnFactor.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSteeringReturnFactor.restype = None
        se.SE_SimpleVehicleSteeringRate.argtypes = [ct.c_void_p, ct.c_double]
        se.SE_SimpleVehicleSteeringRate.restype = None
        se.SE_SimpleVehicleGetState.argtypes = [ct.c_void_p, ct.c_void_p]
        se.SE_SimpleVehicleGetState.restype = None
        se.SE_SimpleVehicleControlAnalog.argtypes = [
            ct.c_void_p,
            ct.c_double,
            ct.c_double,
            ct.c_double,
        ]
        se.SE_SimpleVehicleControlAnalog.restype = None
        se.SE_ReportObjectPosXYH.argtypes = [ct.c_int, ct.c_double, ct.c_double, ct.c_double]
        se.SE_ReportObjectPosXYH.restype = ct.c_int
        se.SE_ReportObjectWheelStatus.argtypes = [ct.c_int, ct.c_double, ct.c_double]
        se.SE_ReportObjectWheelStatus.restype = ct.c_int
        se.SE_ReportObjectSpeed.argtypes = [ct.c_int, ct.c_double]
        se.SE_ReportObjectSpeed.restype = ct.c_int
        se.SE_SetSeed.argtypes = [ct.c_uint]
        se.SE_SetSeed.restype = None
        se.SE_SetLogFilePath.argtypes = [ct.c_char_p]
        se.SE_SetLogFilePath.restype = None
        se.SE_SetWindowPosAndSize.argtypes = [
            ct.c_int,
            ct.c_int,
            ct.c_int,
            ct.c_int,
        ]
        se.SE_SetWindowPosAndSize.restype = None
        se.SE_SetOptionPersistent.argtypes = [ct.c_char_p]
        se.SE_SetOptionPersistent.restype = ct.c_int
        se.SE_SetDatFilePath.argtypes = [ct.c_char_p]
        se.SE_SetDatFilePath.restype = None

    def _setup_esmini_opts(self) -> tuple[int, int, int]:
        self.se.SE_SetSeed(self.options.seed)
        if self.options.log_file_path is not None:
            self.options.log_file_path.parent.mkdir(parents=True, exist_ok=True)
            self.se.SE_SetLogFilePath(str(self.options.log_file_path).encode())
        for path in self.search_paths:
            self.se.SE_AddPath(str(path).encode())
        if self.options.window is not None:
            x, y, width, height = self.options.window
            self.se.SE_SetWindowPosAndSize(x, y, width, height)
        if self.options.disable_stdout:
            self.se.SE_SetOptionPersistent(b"disable_stdout")
        if self.options.dat_file_path is not None:
            self.options.dat_file_path.parent.mkdir(parents=True, exist_ok=True)
            self.se.SE_SetDatFilePath(str(self.options.dat_file_path).encode())
        return (
            self.options.use_viewer,
            self.options.threads,
            self.options.record,
        )

    def start(self, xosc_path: Path, disable_controller: int | None = None) -> None:
        use_viewer, threads, record = self._setup_esmini_opts()
        ret = self.se.SE_Init(
            str(xosc_path).encode(),
            (
                self.options.disable_controller
                if disable_controller is None
                else int(disable_controller)
            ),
            use_viewer,
            threads,
            record,
        )
        if ret != 0:
            raise RuntimeError(f"esmini SE_Init failed, ret={ret}")

        self.obj_id = self.se.SE_GetId(self.ego_index)
        obj_state = self.get_object_state()
        self.ego_length = float(obj_state.length)

    def get_object_state(self) -> SEScenarioObjectState:
        obj_state = SEScenarioObjectState()
        ret = self.se.SE_GetObjectState(self.obj_id, ct.byref(obj_state))
        if ret != 0:
            raise RuntimeError(
                f"SE_GetObjectState failed for ego object id {self.obj_id}"
            )
        return obj_state

    def report_initial_pose(
        self,
        x: float,
        y: float,
        yaw_rad: float,
        speed: float,
    ) -> None:
        self.se.SE_ReportObjectPosXYH(
            self.obj_id,
            float(x),
            float(y),
            float(yaw_rad),
        )
        self.se.SE_ReportObjectWheelStatus(self.obj_id, 0.0, 0.0)
        self.se.SE_ReportObjectSpeed(self.obj_id, float(speed))
        ret = self.se.SE_StepDT(0.0)
        if ret != 0:
            raise RuntimeError(f"SE_StepDT(0.0) failed during initial pose report, ret={ret}")

    def get_ego_state(self, timestamp_s: float) -> VehicleState:
        obj_state = self.get_object_state()
        acceleration = float(self.se.SE_GetObjectAcceleration(self.obj_id))
        speed = float(obj_state.speed)
        yaw = normalize_yaw_rad(float(obj_state.h))
        wheel_angle = float(obj_state.wheel_angle)
        wheel_rotation = float(obj_state.wheel_rot)
        return VehicleState(
            timestamp_s=timestamp_s,
            x=float(obj_state.x),
            y=float(obj_state.y),
            z=float(obj_state.z),
            yaw=yaw,
            speed=speed,
            acceleration=acceleration,
            vel_x=float(speed * math.cos(yaw)),
            vel_y=float(speed * math.sin(yaw)),
            vel_z=0.0,
            ax=float(acceleration * math.cos(yaw)),
            ay=float(acceleration * math.sin(yaw)),
            az=0.0,
            wheel_angle=wheel_angle,
            wheel_rotation=wheel_rotation,
        )

    def should_quit(self) -> bool:
        return bool(self.se.SE_GetQuitFlag())

    def close(self) -> None:
        self.se.SE_Close()
