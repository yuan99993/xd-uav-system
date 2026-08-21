"""ROS-independent YAML component graph and roslaunch generation."""

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Sequence
from xml.etree import ElementTree


@dataclass(frozen=True)
class LaunchComponent:
    component_id: str
    role: str
    stage: str
    package: str
    launch: str
    namespace: str
    args: Mapping[str, str]


def _scalar(value: Any) -> bool:
    return isinstance(value, (str, int, float, bool)) and value is not None


def _enabled_components(profile: Mapping) -> List[Mapping]:
    components = profile.get("components", [])
    if not isinstance(components, list):
        return []
    return [item for item in components
            if isinstance(item, dict) and item.get("enabled", True)]


def validate_component_profile(profile: Mapping) -> Sequence[str]:
    errors: List[str] = []
    if profile.get("schema_version") != 2:
        return ["schema_version must be 2"]

    vehicles = profile.get("vehicles")
    if not isinstance(vehicles, list) or not vehicles:
        errors.append("vehicles must be a non-empty list")
        vehicles = []
    names = []
    for vehicle in vehicles:
        if not isinstance(vehicle, dict) or not vehicle.get("name"):
            errors.append("every vehicle needs a name")
            continue
        names.append(vehicle["name"])
    if len(names) != len(set(names)):
        errors.append("vehicle names must be unique")

    components = profile.get("components")
    if not isinstance(components, list) or not components:
        errors.append("components must be a non-empty list")
        components = []
    ids = []
    for component in components:
        if not isinstance(component, dict):
            errors.append("every component must be a mapping")
            continue
        component_id = component.get("id")
        if not isinstance(component_id, str) or not component_id:
            errors.append("every component needs an id")
        else:
            ids.append(component_id)
        if component.get("scope", "vehicle") not in ("global", "vehicle"):
            errors.append("component {} has invalid scope".format(
                component_id or "<unknown>"))
        if component.get("stage", "base") not in ("base", "runtime"):
            errors.append("component {} has invalid stage".format(
                component_id or "<unknown>"))
        for key in ("role", "package", "launch"):
            if not isinstance(component.get(key), str) or not component[key]:
                errors.append("component {} needs {}".format(
                    component_id or "<unknown>", key))
        if not isinstance(component.get("enabled", True), bool):
            errors.append("component {} enabled must be boolean".format(
                component_id or "<unknown>"))
        args = component.get("args", {})
        if not isinstance(args, dict) or any(
                not isinstance(key, str) or not _scalar(value)
                for key, value in args.items()):
            errors.append("component {} args must contain scalar values".format(
                component_id or "<unknown>"))
        requires = component.get("requires", [])
        if not isinstance(requires, list) or any(
                not isinstance(item, str) for item in requires):
            errors.append("component {} requires must be a string list".format(
                component_id or "<unknown>"))
    if len(ids) != len(set(ids)):
        errors.append("component ids must be unique")

    enabled = _enabled_components(profile)
    enabled_ids = {item.get("id") for item in enabled}
    enabled_by_id = {item.get("id"): item for item in enabled}
    stage_order = {"base": 0, "runtime": 1}
    for component in enabled:
        for dependency in component.get("requires", []):
            if dependency not in enabled_ids:
                errors.append("component {} requires enabled component {}".format(
                    component.get("id"), dependency))
            elif stage_order[enabled_by_id[dependency].get("stage", "base")] > \
                    stage_order[component.get("stage", "base")]:
                errors.append("component {} depends on later-stage component {}".format(
                    component.get("id"), dependency))

    roles = [item.get("role") for item in enabled]
    if roles.count("reference_arbiter") != 1:
        errors.append("exactly one enabled reference_arbiter is required")
    if roles.count("controller") != 1:
        errors.append("exactly one enabled controller is required")

    control = profile.get("control", {})
    owner = control.get("initial_owner") if isinstance(control, dict) else None
    if owner not in ("none", "manual", "sead", "ego"):
        errors.append("control.initial_owner must be none, manual, sead or ego")
    source_roles = {
        item.get("source") for item in enabled
        if item.get("role") == "reference_source"
    }
    if owner != "none" and owner not in source_roles:
        errors.append("initial_owner requires an enabled reference_source")

    simulation = profile.get("simulation", {})
    backend = simulation.get("backend") if isinstance(simulation, dict) else None
    sensing_types = {item.get("type") for item in enabled
                     if item.get("role") == "sensing"}
    if backend == "px4_gazebo" and "fake_drone" in sensing_types:
        errors.append("px4_gazebo and fake_drone are mutually exclusive")
    return errors


def _format(value: Any, context: Mapping[str, Any]) -> str:
    result = str(value)
    for key, replacement in context.items():
        result = result.replace("{" + key + "}", str(replacement))
    if "{" in result or "}" in result:
        raise ValueError("unresolved profile placeholder: {}".format(result))
    return result


def build_launch_plan(profile: Mapping) -> Sequence[LaunchComponent]:
    errors = validate_component_profile(profile)
    if errors:
        raise ValueError("; ".join(errors))
    frames = profile.get("frames", {})
    common = frames.get("common", "world")
    plan: List[LaunchComponent] = []
    for component in _enabled_components(profile):
        targets: Iterable[Mapping]
        if component.get("scope", "vehicle") == "global":
            targets = ({},)
        else:
            targets = profile["vehicles"]
        for vehicle in targets:
            context: Dict[str, Any] = {"frames.common": common}
            context.update({"vehicle." + key: value
                            for key, value in vehicle.items()})
            args = {key: _format(value, context)
                    for key, value in component.get("args", {}).items()}
            component_id = component["id"]
            if vehicle:
                component_id += ":" + str(vehicle["name"])
            plan.append(LaunchComponent(
                component_id=component_id,
                role=component["role"],
                stage=component.get("stage", "base"),
                package=component["package"],
                launch=component["launch"],
                namespace=_format(component.get("namespace", ""), context),
                args=args))
    return plan


def render_roslaunch(plan: Sequence[LaunchComponent], stage: str = "base") -> str:
    if stage not in ("base", "runtime", "all"):
        raise ValueError("stage must be base, runtime or all")
    root = ElementTree.Element("launch")
    root.append(ElementTree.Comment(
        " Generated from a validated xd_uav_system_integration profile. "))
    for component in plan:
        if stage != "all" and component.stage != stage:
            continue
        include = ElementTree.SubElement(root, "include", {
            "file": "$(find {})/launch/{}".format(
                component.package, component.launch),
        })
        if component.namespace:
            include.set("ns", component.namespace)
        for name in sorted(component.args):
            ElementTree.SubElement(include, "arg", {
                "name": name,
                "value": component.args[name],
            })
    # ElementTree.indent() was added in Python 3.9; ROS Noetic uses 3.8.
    def indent(element, level=0):
        padding = "\n" + level * "  "
        child_padding = "\n" + (level + 1) * "  "
        if len(element):
            if not element.text or not element.text.strip():
                element.text = child_padding
            for child in element:
                indent(child, level + 1)
                if not child.tail or not child.tail.strip():
                    child.tail = child_padding
            child.tail = padding
    indent(root)
    return ElementTree.tostring(root, encoding="unicode") + "\n"
