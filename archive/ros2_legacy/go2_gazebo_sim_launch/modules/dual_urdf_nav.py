"""Dual-robot URDF builder for nav-only scenes (no door barrier).

Variant of ``modules.dual_urdf.build_dual_mujoco_urdf`` that omits the
door-barrier ``ros2_control`` block + stub links. Used by dual-robot nav
benchmarks (demo3_dual.xml) where the MJCF has no door_barrier_slide
joint — adding the block anyway causes mujoco_ros2_control to log a
WARN about an unknown joint target and increases DDS chatter.
"""
from __future__ import annotations

from xml.dom import minidom

_PREFIX = "b_"


def _clone_element(elem, prefix: str):
    clone = elem.cloneNode(deep=True)
    _NAME_ATTRS = {"name"}
    _LINK_REF_TAGS = {"parent", "child"}
    for node in [clone] + list(clone.getElementsByTagName("*")):
        for attr in list(node.attributes.keys()) if node.attributes else []:
            if attr in _NAME_ATTRS:
                node.setAttribute(attr, prefix + node.getAttribute(attr))
        if node.tagName in _LINK_REF_TAGS and node.hasAttribute("link"):
            node.setAttribute("link", prefix + node.getAttribute("link"))
    return clone


def _prefix_ros2_control_joints(ros2_control_elem, prefix: str, new_name: str):
    clone = ros2_control_elem.cloneNode(deep=True)
    clone.setAttribute("name", new_name)
    for joint_el in clone.getElementsByTagName("joint"):
        if joint_el.hasAttribute("name"):
            joint_el.setAttribute("name", prefix + joint_el.getAttribute("name"))
    return clone


def build_dual_nav_urdf(base_urdf_string: str) -> str:
    """Combined URDF with two robots, no door barrier.

    Clones all ``<link>`` / ``<joint>`` under a ``b_`` prefix, adds
    fixed world→base_link and world→b_base_link joints, duplicates the
    ``<ros2_control>`` block as ``robot_a_system`` / ``robot_b_system``,
    and swaps Gazebo plugin → MujocoSystem.
    """
    doc = minidom.parseString(base_urdf_string)
    robot = doc.documentElement

    world_link = doc.createElement("link")
    world_link.setAttribute("name", "world")
    robot.appendChild(doc.createTextNode("\n  "))
    robot.appendChild(world_link)

    root_link_name = "base_link"

    fix_a = doc.createElement("joint")
    fix_a.setAttribute("name", "world_to_robot_a")
    fix_a.setAttribute("type", "fixed")
    parent_a = doc.createElement("parent"); parent_a.setAttribute("link", "world")
    child_a = doc.createElement("child"); child_a.setAttribute("link", root_link_name)
    fix_a.appendChild(parent_a); fix_a.appendChild(child_a)
    robot.appendChild(doc.createTextNode("\n  ")); robot.appendChild(fix_a)

    links = [e for e in robot.childNodes
             if e.nodeType == e.ELEMENT_NODE and e.tagName == "link"
             and e.getAttribute("name") != "world"]
    joints = [e for e in robot.childNodes
              if e.nodeType == e.ELEMENT_NODE and e.tagName == "joint"
              and e.getAttribute("name") != "world_to_robot_a"]
    for link in links:
        robot.appendChild(doc.createTextNode("\n  "))
        robot.appendChild(_clone_element(link, _PREFIX))
    for joint in joints:
        robot.appendChild(doc.createTextNode("\n  "))
        robot.appendChild(_clone_element(joint, _PREFIX))

    fix_b = doc.createElement("joint")
    fix_b.setAttribute("name", "world_to_robot_b")
    fix_b.setAttribute("type", "fixed")
    parent_b = doc.createElement("parent"); parent_b.setAttribute("link", "world")
    child_b = doc.createElement("child"); child_b.setAttribute("link", _PREFIX + root_link_name)
    fix_b.appendChild(parent_b); fix_b.appendChild(child_b)
    robot.appendChild(doc.createTextNode("\n  ")); robot.appendChild(fix_b)

    ros2_blocks = robot.getElementsByTagName("ros2_control")
    if ros2_blocks:
        original = ros2_blocks[0]
        original.setAttribute("name", "robot_a_system")
        robot_b_block = _prefix_ros2_control_joints(original, _PREFIX, "robot_b_system")
        original.parentNode.insertBefore(doc.createTextNode("\n  "), original.nextSibling)
        original.parentNode.insertBefore(robot_b_block, original.nextSibling.nextSibling)

    for plugin in doc.getElementsByTagName("plugin"):
        for child in plugin.childNodes:
            if child.nodeType == child.TEXT_NODE and "GazeboSystem" in child.data:
                child.data = child.data.replace(
                    "gazebo_ros2_control/GazeboSystem",
                    "mujoco_ros2_control/MujocoSystem",
                )

    for gazebo in list(doc.getElementsByTagName("gazebo")):
        has_gazebo_plugin = False
        for plugin in gazebo.getElementsByTagName("plugin"):
            fname = plugin.getAttribute("filename") or ""
            if "libgazebo" in fname:
                has_gazebo_plugin = True
                break
        if has_gazebo_plugin:
            gazebo.parentNode.removeChild(gazebo)

    return doc.toxml()
