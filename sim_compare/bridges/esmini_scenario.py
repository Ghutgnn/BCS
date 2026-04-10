from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from sim_compare.models import InitialPose


def _find_or_create(parent: ET.Element, tag: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    return child


def _find_private_init(actions: ET.Element, entity_ref: str) -> ET.Element:
    for private in actions.findall("Private"):
        if private.get("entityRef") == entity_ref:
            return private
    return ET.SubElement(actions, "Private", {"entityRef": entity_ref})


def _split_private_actions(
    private: ET.Element,
    tag_suffixes: tuple[str, ...],
) -> list[ET.Element]:
    kept_actions: list[ET.Element] = []
    for private_action in list(private.findall("PrivateAction")):
        private.remove(private_action)
        if not any(
            private_action.find(suffix) is not None
            or any(child.tag == suffix for child in list(private_action))
            for suffix in tag_suffixes
        ):
            kept_actions.append(private_action)
    return kept_actions


def read_front_axle_max_steering_rad(
    xosc_path: Path,
    ego_name: str = "Ego",
    default_rad: float = math.radians(30.0),
) -> float:
    tree = ET.parse(xosc_path)
    root = tree.getroot()
    scenario_object = None
    for candidate in root.findall("./Entities/ScenarioObject"):
        if candidate.get("name") == ego_name:
            scenario_object = candidate
            break
    if scenario_object is None:
        return default_rad

    axle = scenario_object.find("./Vehicle/Axles/FrontAxle")
    if axle is None:
        return default_rad

    value_text = axle.get("maxSteering")
    if not value_text:
        return default_rad

    value = float(value_text)
    if abs(value) > math.pi:
        return math.radians(value)
    return value


def build_bcs_runtime_xosc(
    base_xosc_path: Path,
    output_path: Path,
    initial_pose: InitialPose,
    base_port: int,
    exec_mode: str,
    ego_name: str = "Ego",
) -> Path:
    tree = ET.parse(base_xosc_path)
    root = tree.getroot()

    entities = root.find("Entities")
    if entities is None:
        raise ValueError(f"Missing Entities section in {base_xosc_path}")

    ego_object = None
    for scenario_object in entities.findall("ScenarioObject"):
        if scenario_object.get("name") == ego_name:
            ego_object = scenario_object
            break
    if ego_object is None:
        raise ValueError(f"ScenarioObject '{ego_name}' not found in {base_xosc_path}")

    object_controller = ego_object.find("ObjectController")
    if object_controller is None:
        object_controller = ET.SubElement(ego_object, "ObjectController")
    object_controller.clear()
    controller = ET.SubElement(
        object_controller, "Controller", {"name": "RuntimeBCSController"}
    )
    properties = ET.SubElement(controller, "Properties")
    ET.SubElement(
        properties,
        "Property",
        {"name": "esminiController", "value": "BCSController"},
    )
    ET.SubElement(properties, "Property", {"name": "inputMode", "value": "driverInput"})
    ET.SubElement(properties, "Property", {"name": "basePort", "value": str(base_port)})
    ET.SubElement(properties, "Property", {"name": "port", "value": "0"})
    ET.SubElement(properties, "Property", {"name": "execMode", "value": exec_mode})

    storyboard = _find_or_create(root, "Storyboard")
    init_elem = _find_or_create(storyboard, "Init")
    actions = _find_or_create(init_elem, "Actions")
    private = _find_private_init(actions, ego_name)
    kept_actions = _split_private_actions(
        private,
        (
            "TeleportAction",
            "LongitudinalAction",
            "ControllerAction",
            "ActivateControllerAction",
        ),
    )
    for kept_action in kept_actions:
        private.append(kept_action)

    yaw_rad = math.radians(initial_pose.yaw_deg)
    teleport = ET.SubElement(private, "PrivateAction")
    teleport_action = ET.SubElement(teleport, "TeleportAction")
    position = ET.SubElement(teleport_action, "Position")
    ET.SubElement(
        position,
        "WorldPosition",
        {
            "x": f"{initial_pose.x:.9f}",
            "y": f"{initial_pose.y:.9f}",
            "z": "0.0",
            "h": f"{yaw_rad:.9f}",
            "p": "0.0",
            "r": "0.0",
        },
    )

    speed_action = ET.SubElement(private, "PrivateAction")
    longitudinal = ET.SubElement(speed_action, "LongitudinalAction")
    speed = ET.SubElement(longitudinal, "SpeedAction")
    ET.SubElement(
        speed,
        "SpeedActionDynamics",
        {
            "dynamicsShape": "step",
            "value": "0",
            "dynamicsDimension": "time",
        },
    )
    target = ET.SubElement(speed, "SpeedActionTarget")
    ET.SubElement(
        target,
        "AbsoluteTargetSpeed",
        {"value": f"{initial_pose.speed:.9f}"},
    )

    activate_action = ET.SubElement(private, "PrivateAction")
    controller_action = ET.SubElement(activate_action, "ControllerAction")
    ET.SubElement(
        controller_action,
        "ActivateControllerAction",
        {"longitudinal": "true", "lateral": "true"},
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


# Backward-compatible alias for older integration code.
build_udp_driver_runtime_xosc = build_bcs_runtime_xosc
