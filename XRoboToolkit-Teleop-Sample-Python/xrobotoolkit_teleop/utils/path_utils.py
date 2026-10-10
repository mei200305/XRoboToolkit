import os
import re
import tempfile
from copy import deepcopy
from pathlib import Path
from typing import Dict, Optional
import xml.etree.ElementTree as ET

PACKAGE_PATH = os.path.dirname(os.path.abspath(__file__))
ASSET_PATH = os.path.join(PACKAGE_PATH, "../../assets")

# Default teleop URDF for QJ2M: standard (non-extended) arms, from ~/Code/2m qj2m_description.
# assets/qj2m is the "20260805 加长臂" extended-arm model and does not match the robot.
QJ2M_URDF_PATH = os.path.join(ASSET_PATH, "qj2m_short_arm/urdf/qj2m_teleop.urdf")


def resolve_ros_package_uris_in_urdf(
    urdf_path: str, package_map: Optional[Dict[str, str]] = None
) -> str:
    """
    Replace `package://<pkg>/...` mesh URIs in a URDF with absolute filesystem paths.

    This helps libraries like placo that do not resolve ROS package URIs on their own.
    Returns the original path when no replacement is needed.
    """
    urdf_file = Path(urdf_path).expanduser().resolve()
    content = urdf_file.read_text(encoding="utf-8")

    if "package://" not in content:
        return str(urdf_file)

    resolved_package_map = {}
    if package_map:
        resolved_package_map.update({name: str(Path(path).expanduser().resolve()) for name, path in package_map.items()})

    # Common ROS layout: <pkg>/urdf/file.urdf
    if urdf_file.parent.name == "urdf":
        candidate_pkg_dir = urdf_file.parent.parent
        pkg_dir_str = str(candidate_pkg_dir)
        resolved_package_map.setdefault(candidate_pkg_dir.name, pkg_dir_str)
        # SolidWorks exporters often use assembly name as package://, not folder name
        for pkg_name in set(re.findall(r"package://([^/]+)/", content)):
            resolved_package_map.setdefault(pkg_name, pkg_dir_str)

    updated_content = content
    for package_name, package_dir in resolved_package_map.items():
        updated_content = updated_content.replace(f"package://{package_name}/", f"{package_dir}/")

    if updated_content == content:
        return str(urdf_file)

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=f"_{urdf_file.name}",
        prefix="resolved_urdf_",
        delete=False,
        encoding="utf-8",
    ) as tmp_file:
        tmp_file.write(updated_content)
        return tmp_file.name


def build_dual_arm_urdf_from_single(
    single_urdf_path: str,
    left_prefix: str = "left_",
    right_prefix: str = "right_",
    left_mount_xyz: str = "0 0.35 0",
    right_mount_xyz: str = "0 -0.35 0",
    left_mount_rpy: str = "0 0 0",
    right_mount_rpy: str = "0 0 0",
) -> str:
    """
    Build a temporary dual-arm URDF by duplicating a single-arm URDF twice.

    The source URDF is expected to have a rooted fixed chain beginning at `dummy_link`.
    """
    resolved_single_urdf_path = resolve_ros_package_uris_in_urdf(single_urdf_path)
    source_root = ET.parse(resolved_single_urdf_path).getroot()

    dual_root = ET.Element("robot", {"name": f"{source_root.attrib.get('name', 'robot')}_dual"})
    ET.SubElement(dual_root, "link", {"name": "world"})

    source_links = source_root.findall("link")
    source_joints = source_root.findall("joint")

    def prefixed(name: str, prefix: str) -> str:
        return f"{prefix}{name}"

    def append_arm(prefix: str, mount_xyz: str, mount_rpy: str):
        for link in source_links:
            new_link = deepcopy(link)
            new_link.attrib["name"] = prefixed(link.attrib["name"], prefix)
            dual_root.append(new_link)

        for joint in source_joints:
            new_joint = deepcopy(joint)
            new_joint.attrib["name"] = prefixed(joint.attrib["name"], prefix)

            parent = new_joint.find("parent")
            if parent is not None:
                parent.attrib["link"] = prefixed(parent.attrib["link"], prefix)

            child = new_joint.find("child")
            if child is not None:
                child.attrib["link"] = prefixed(child.attrib["link"], prefix)

            dual_root.append(new_joint)

        mount_joint = ET.SubElement(
            dual_root,
            "joint",
            {
                "name": f"{prefix}world_mount",
                "type": "fixed",
            },
        )
        ET.SubElement(mount_joint, "origin", {"xyz": mount_xyz, "rpy": mount_rpy})
        ET.SubElement(mount_joint, "parent", {"link": "world"})
        ET.SubElement(mount_joint, "child", {"link": prefixed("dummy_link", prefix)})

    append_arm(left_prefix, left_mount_xyz, left_mount_rpy)
    append_arm(right_prefix, right_mount_xyz, right_mount_rpy)

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix="_dual.urdf",
        prefix="generated_dual_urdf_",
        delete=False,
        encoding="utf-8",
    ) as tmp_file:
        ET.ElementTree(dual_root).write(tmp_file, encoding="unicode", xml_declaration=True)
        return tmp_file.name


def namespaced_topic(namespace: str, topic: str) -> str:
    namespace = (namespace or "").strip()
    topic = topic.strip()

    if not topic.startswith("/"):
        topic = f"/{topic}"
    if not namespace:
        return topic

    if not namespace.startswith("/"):
        namespace = f"/{namespace}"
    return f"{namespace.rstrip('/')}{topic}"
