"""Generate a combined dual-robot URDF for mujoco_ros2_control.

The MJCF door scene (two_rooms_door_scene.xml) embeds both robots in one file.
Robot A uses standard joint/link names; Robot B uses a ``b_`` prefix.
mujoco_ros2_control requires a single ``robot_description`` URDF whose
``<ros2_control>`` blocks and ``<joint>`` / ``<link>`` elements match the MJCF.

This module:
1. Takes the processed Robot A xacro URDF.
2. Clones every ``<link>`` and ``<joint>`` with a ``b_`` prefix → Robot B.
3. Adds a second ``<ros2_control>`` block for Robot B's joints.
4. Replaces the hardware plugin with MujocoSystem.
5. Returns the combined URDF string for mujoco_ros2_control,
   plus per-robot URDF strings for each robot_state_publisher.
"""

from __future__ import annotations

import copy
import re
from xml.dom import minidom


_PREFIX = "b_"


def _clone_element(elem, prefix: str):
    """Deep-clone a DOM element and prefix all ``name`` / link-reference attrs."""
    clone = elem.cloneNode(deep=True)

    # Attributes that hold link or joint names
    _NAME_ATTRS = {"name"}
    # <parent link="..."/> and <child link="..."/> inside <joint> elements
    _LINK_REF_TAGS = {"parent", "child"}

    for node in [clone] + list(clone.getElementsByTagName("*")):
        for attr in list(node.attributes.keys()) if node.attributes else []:
            if attr in _NAME_ATTRS:
                node.setAttribute(attr, prefix + node.getAttribute(attr))

        if node.tagName in _LINK_REF_TAGS and node.hasAttribute("link"):
            node.setAttribute("link", prefix + node.getAttribute("link"))

    return clone


def _prefix_ros2_control_joints(ros2_control_elem, prefix: str, new_name: str):
    """Clone a ``<ros2_control>`` block and prefix all joint names."""
    clone = ros2_control_elem.cloneNode(deep=True)
    clone.setAttribute("name", new_name)

    for joint_el in clone.getElementsByTagName("joint"):
        if joint_el.hasAttribute("name"):
            joint_el.setAttribute("name", prefix + joint_el.getAttribute("name"))

    return clone


def build_dual_mujoco_urdf(base_urdf_string: str) -> str:
    """Return a combined URDF with both robots for ``mujoco_ros2_control``.

    The hardware plugin is switched to ``mujoco_ros2_control/MujocoSystem``.
    """
    doc = minidom.parseString(base_urdf_string)
    robot = doc.documentElement

    # ── 0. Add a dummy world root link to unify two kinematic trees ──
    # URDF requires exactly one root link; we connect both robots via fixed joints.
    world_link = doc.createElement("link")
    world_link.setAttribute("name", "world")
    robot.appendChild(doc.createTextNode("\n  "))
    robot.appendChild(world_link)

    # Identify Robot A root link (the current root)
    root_link_name = "base_link"
    for link_el in robot.getElementsByTagName("link"):
        if link_el.getAttribute("name") == root_link_name:
            break

    # Fixed joint: world → base_link (Robot A)
    fix_a = doc.createElement("joint")
    fix_a.setAttribute("name", "world_to_robot_a")
    fix_a.setAttribute("type", "fixed")
    parent_a = doc.createElement("parent")
    parent_a.setAttribute("link", "world")
    child_a = doc.createElement("child")
    child_a.setAttribute("link", root_link_name)
    fix_a.appendChild(parent_a)
    fix_a.appendChild(child_a)
    robot.appendChild(doc.createTextNode("\n  "))
    robot.appendChild(fix_a)

    # ── 1. Clone links and joints for Robot B ──
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

    # Fixed joint: world → b_base_link (Robot B)
    fix_b = doc.createElement("joint")
    fix_b.setAttribute("name", "world_to_robot_b")
    fix_b.setAttribute("type", "fixed")
    parent_b = doc.createElement("parent")
    parent_b.setAttribute("link", "world")
    child_b = doc.createElement("child")
    child_b.setAttribute("link", _PREFIX + root_link_name)
    fix_b.appendChild(parent_b)
    fix_b.appendChild(child_b)
    robot.appendChild(doc.createTextNode("\n  "))
    robot.appendChild(fix_b)

    # ��─ 2. Duplicate <ros2_control> block for Robot B ──
    ros2_blocks = robot.getElementsByTagName("ros2_control")
    if ros2_blocks:
        original = ros2_blocks[0]
        original.setAttribute("name", "robot_a_system")
        robot_b_block = _prefix_ros2_control_joints(original, _PREFIX, "robot_b_system")
        original.parentNode.insertBefore(doc.createTextNode("\n  "), original.nextSibling)
        original.parentNode.insertBefore(robot_b_block, original.nextSibling.nextSibling)

        # Third ros2_control block for the door-lock barrier slide
        # joint. The barrier is a separate MJCF body that slides into
        # and out of the doorway; a forward_command_controller writes
        # its position target. The door_hinge joint itself is NOT
        # exposed to ros2_control — it runs purely under the FD30
        # spring model in MJCF.
        door_block = doc.createElement("ros2_control")
        door_block.setAttribute("name", "door_system")
        door_block.setAttribute("type", "system")
        door_hw = doc.createElement("hardware")
        door_plugin = doc.createElement("plugin")
        door_plugin.appendChild(
            doc.createTextNode("mujoco_ros2_control/MujocoSystem")
        )
        door_hw.appendChild(door_plugin)
        door_block.appendChild(door_hw)
        door_joint = doc.createElement("joint")
        door_joint.setAttribute("name", "door_barrier_slide")
        eff_cmd = doc.createElement("command_interface")
        eff_cmd.setAttribute("name", "position")
        door_joint.appendChild(eff_cmd)
        for sname in ("position", "velocity"):
            si = doc.createElement("state_interface")
            si.setAttribute("name", sname)
            door_joint.appendChild(si)
        door_block.appendChild(door_joint)
        robot.appendChild(doc.createTextNode("\n  "))
        robot.appendChild(door_block)

        # mujoco_ros2_control also looks up the joint in the URDF
        # kinematic tree. Add stub links + a fixed connector to the
        # main base_link so the URDF remains a single connected tree;
        # then a prismatic joint named door_barrier_slide between
        # the stubs. Physics come from MJCF, so the stubs are empty.
        stub_anchor = doc.createElement("link")
        stub_anchor.setAttribute("name", "door_stub_anchor")
        stub_panel = doc.createElement("link")
        stub_panel.setAttribute("name", "door_stub_panel")

        # Fixed joint attaching the anchor to base_link (robot A root).
        stub_fix = doc.createElement("joint")
        stub_fix.setAttribute("name", "door_stub_fix")
        stub_fix.setAttribute("type", "fixed")
        sp = doc.createElement("parent")
        sp.setAttribute("link", "base_link")
        sc = doc.createElement("child")
        sc.setAttribute("link", "door_stub_anchor")
        so = doc.createElement("origin")
        so.setAttribute("xyz", "0 0 0")
        so.setAttribute("rpy", "0 0 0")
        stub_fix.appendChild(sp)
        stub_fix.appendChild(sc)
        stub_fix.appendChild(so)

        # The actual barrier kinematic joint (stubbed). prismatic,
        # limited — matches the MJCF slide joint limits.
        door_kin = doc.createElement("joint")
        door_kin.setAttribute("name", "door_barrier_slide")
        door_kin.setAttribute("type", "prismatic")
        kp = doc.createElement("parent")
        kp.setAttribute("link", "door_stub_anchor")
        kc = doc.createElement("child")
        kc.setAttribute("link", "door_stub_panel")
        ko = doc.createElement("origin")
        ko.setAttribute("xyz", "0 0 0")
        ko.setAttribute("rpy", "0 0 0")
        ka = doc.createElement("axis")
        ka.setAttribute("xyz", "0 0 1")
        klim = doc.createElement("limit")
        # Must match the MJCF `<joint range="-0.1 3.1">` and the
        # position actuator's ctrlrange. mujoco_ros2_control clamps
        # position commands to the URDF limit, so any mismatch silently
        # truncates the target — that's what happened on 2026-04-14
        # when the barrier was supposed to retract to +3.0 but the old
        # URDF limit upper=0.1 clamped it to 0.1 instead.
        klim.setAttribute("lower", "-0.1")
        klim.setAttribute("upper", "3.1")
        klim.setAttribute("effort", "1000000")
        klim.setAttribute("velocity", "100")
        door_kin.appendChild(kp)
        door_kin.appendChild(kc)
        door_kin.appendChild(ko)
        door_kin.appendChild(ka)
        door_kin.appendChild(klim)

        for node in (stub_anchor, stub_panel, stub_fix, door_kin):
            robot.appendChild(doc.createTextNode("\n  "))
            robot.appendChild(node)

    # ── 3. Replace hardware plugin with MujocoSystem ──
    for plugin in doc.getElementsByTagName("plugin"):
        text_nodes = [c for c in plugin.childNodes if c.nodeType == c.TEXT_NODE]
        for t in text_nodes:
            if "GazeboSystem" in t.data:
                t.data = t.data.replace("gazebo_ros2_control/GazeboSystem",
                                        "mujoco_ros2_control/MujocoSystem")
        if plugin.hasChildNodes() and not text_nodes:
            # plugin text might be in the element itself
            pass

    # Also handle the case where the plugin name is an attribute
    for hw in doc.getElementsByTagName("hardware"):
        for plugin in hw.getElementsByTagName("plugin"):
            # Text content
            for child in plugin.childNodes:
                if child.nodeType == child.TEXT_NODE and "GazeboSystem" in child.data:
                    child.data = child.data.replace(
                        "gazebo_ros2_control/GazeboSystem",
                        "mujoco_ros2_control/MujocoSystem",
                    )

    # ─��� 4. Remove Gazebo-only plugin blocks (they crash mujoco_ros2_control) ─���
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


def build_mixed_mujoco_urdf(go2w_urdf_string: str, go2_urdf_string: str) -> str:
    """Combined URDF for mujoco_ros2_control: Go2W (robot_a) + Go2 (robot_b, b_-prefixed).

    Unlike ``build_dual_mujoco_urdf`` (which clones robot_a for robot_b), this takes
    TWO separate URDFs — Go2W for the wheeled robot_a and Go2 for the legged-only
    robot_b — and merges them into one URDF. The Go2 URDF contributes its own
    ``<ros2_control>`` block (12 leg joints, no foot_joint) so mujoco_ros2_control
    binds robot_b to the 12 MJCF leg motors only.
    """
    doc = minidom.parseString(go2w_urdf_string)
    robot = doc.documentElement

    # World dummy root + fixed joint world -> base_link (robot_a)
    world_link = doc.createElement("link")
    world_link.setAttribute("name", "world")
    robot.appendChild(doc.createTextNode("\n  "))
    robot.appendChild(world_link)

    root_link_name = "base_link"
    fix_a = doc.createElement("joint")
    fix_a.setAttribute("name", "world_to_robot_a")
    fix_a.setAttribute("type", "fixed")
    pa = doc.createElement("parent"); pa.setAttribute("link", "world")
    ca = doc.createElement("child");  ca.setAttribute("link", root_link_name)
    fix_a.appendChild(pa); fix_a.appendChild(ca)
    robot.appendChild(doc.createTextNode("\n  "))
    robot.appendChild(fix_a)

    # Rename robot_a's ros2_control block (from Go2W xacro) to robot_a_system
    ros2_blocks = robot.getElementsByTagName("ros2_control")
    if ros2_blocks:
        ros2_blocks[0].setAttribute("name", "robot_a_system")

    # Clone Go2 URDF's links/joints with b_ prefix into the combined doc
    go2_doc = minidom.parseString(go2_urdf_string)
    go2_robot = go2_doc.documentElement

    go2_links = [e for e in go2_robot.childNodes
                 if e.nodeType == e.ELEMENT_NODE and e.tagName == "link"]
    go2_joints = [e for e in go2_robot.childNodes
                  if e.nodeType == e.ELEMENT_NODE and e.tagName == "joint"]
    for link in go2_links:
        robot.appendChild(doc.createTextNode("\n  "))
        robot.appendChild(_clone_element(link, _PREFIX))
    for joint in go2_joints:
        robot.appendChild(doc.createTextNode("\n  "))
        robot.appendChild(_clone_element(joint, _PREFIX))

    # Clone Go2's ros2_control block (12 leg joints, no foot_joint) as robot_b_system
    go2_ros2_blocks = go2_robot.getElementsByTagName("ros2_control")
    if go2_ros2_blocks:
        robot_b_block = _prefix_ros2_control_joints(
            go2_ros2_blocks[0], _PREFIX, "robot_b_system"
        )
        robot.appendChild(doc.createTextNode("\n  "))
        robot.appendChild(robot_b_block)

    # Fixed joint world -> b_base_link
    fix_b = doc.createElement("joint")
    fix_b.setAttribute("name", "world_to_robot_b")
    fix_b.setAttribute("type", "fixed")
    pb = doc.createElement("parent"); pb.setAttribute("link", "world")
    cb = doc.createElement("child");  cb.setAttribute("link", _PREFIX + root_link_name)
    fix_b.appendChild(pb); fix_b.appendChild(cb)
    robot.appendChild(doc.createTextNode("\n  "))
    robot.appendChild(fix_b)

    # Swap GazeboSystem -> MujocoSystem in every <plugin> text node
    for hw in doc.getElementsByTagName("hardware"):
        for plugin in hw.getElementsByTagName("plugin"):
            for child in plugin.childNodes:
                if child.nodeType == child.TEXT_NODE and "GazeboSystem" in child.data:
                    child.data = child.data.replace(
                        "gazebo_ros2_control/GazeboSystem",
                        "mujoco_ros2_control/MujocoSystem",
                    )

    # Strip Gazebo plugin blocks (they crash mujoco_ros2_control)
    for gazebo in list(doc.getElementsByTagName("gazebo")):
        has_gazebo_plugin = any(
            "libgazebo" in (p.getAttribute("filename") or "")
            for p in gazebo.getElementsByTagName("plugin")
        )
        if has_gazebo_plugin:
            gazebo.parentNode.removeChild(gazebo)

    return doc.toxml()


def build_robot_b_urdf(base_urdf_string: str) -> str:
    """Return a Robot B URDF (b_-prefixed joints/links) for its robot_state_publisher.

    This keeps the kinematic tree intact but prefixes every joint and link name.
    The ``<ros2_control>`` and ``<gazebo>`` blocks are removed (RSP doesn't need them).
    """
    doc = minidom.parseString(base_urdf_string)
    robot = doc.documentElement

    # Prefix all link and joint names
    for link in robot.getElementsByTagName("link"):
        if link.hasAttribute("name"):
            link.setAttribute("name", _PREFIX + link.getAttribute("name"))
    for joint in robot.getElementsByTagName("joint"):
        if joint.hasAttribute("name"):
            joint.setAttribute("name", _PREFIX + joint.getAttribute("name"))
    for parent in robot.getElementsByTagName("parent"):
        if parent.hasAttribute("link"):
            parent.setAttribute("link", _PREFIX + parent.getAttribute("link"))
    for child_el in robot.getElementsByTagName("child"):
        if child_el.hasAttribute("link"):
            child_el.setAttribute("link", _PREFIX + child_el.getAttribute("link"))

    # Remove ros2_control and gazebo blocks (RSP doesn't need them)
    for tag in ("ros2_control", "gazebo"):
        for elem in list(robot.getElementsByTagName(tag)):
            elem.parentNode.removeChild(elem)

    return doc.toxml()
