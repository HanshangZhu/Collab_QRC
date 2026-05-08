#!/usr/bin/env python3
"""Race-tolerant replacement for ``ros2 run controller_manager spawner``.

Two known failure modes it handles:

1. **Upstream spawner's per-call timeout is too short.** The stock spawner
   hard-codes 10 s for every service call. During MuJoCo startup the
   controller_manager is pegged on physics/mesh init and may take longer
   to return from ``load_controller``. The stock spawner then retries,
   hitting "already loaded" from its own successful-but-late first call,
   and dies. The controller stays ``UNCONFIGURED``, ``stand_up_slowly``
   blocks forever.

2. **Responses dropped under rmw congestion.** The CM may *process* a
   request but fail to deliver the response ("failed to send response to
   /controller_manager/... (timeout): client will not receive response").
   We use short per-call timeouts with retries, and after each call we
   verify state via ``list_controllers`` — so a dropped response is not
   catastrophic, just a prompt to check and retry.

CLI mirrors the upstream spawner's flags we actually use: positional
controller name, ``--controller-manager``, ``--controller-manager-timeout``,
``--inactive``. Unknown args are swallowed.

Design note: unlike upstream, we do **not** check list_controllers first
as an optimization — that very first call is what hangs under rmw
congestion (the joint_states race wins; the effort one loses). We just
call load_controller, and if the request returns "ok=false" or the
response is lost, we use list_controllers to recover.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.node import Node

# Per-service-call timeout. Keep this modest (we retry). Upstream is 10 s
# and retries 3× with its own stricter "already loaded" semantics; we take
# any single-call stall as a signal to verify state and move on.
CALL_TIMEOUT_SEC = 8.0
MAX_ATTEMPTS = 6


def _wait_service(node: Node, client, timeout: float) -> bool:
    deadline = time.time() + timeout
    while not client.wait_for_service(timeout_sec=0.5):
        if time.time() > deadline:
            return False
    return True


def _call(node: Node, client, req, timeout: float):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.done():
        return fut.result()
    # Future never completed — cancel it so rclpy doesn't keep holding the
    # response path open, then return None to signal retry.
    client.remove_pending_request(fut)
    return None


def _list(node: Node, list_c, timeout: float):
    return _call(node, list_c, ListControllers.Request(), timeout)


def _state_of(node: Node, list_c, name: str, timeout: float) -> str | None:
    """Return the current state string, or None if not loaded / list failed."""
    res = _list(node, list_c, timeout)
    if res is None:
        return None
    for c in res.controller:
        if c.name == name:
            return c.state
    return ""  # found list, but controller not present


def load(node: Node, load_c, list_c, name: str) -> bool:
    for attempt in range(1, MAX_ATTEMPTS + 1):
        res = _call(node, load_c, LoadController.Request(name=name),
                    CALL_TIMEOUT_SEC)
        if res is not None and res.ok:
            node.get_logger().info(f"loaded {name} (attempt {attempt})")
            return True
        # Either res.ok == False ("already loaded"-style), or the response
        # was dropped (res is None). Both point us to list_controllers to
        # verify actual state.
        state = _state_of(node, list_c, name, CALL_TIMEOUT_SEC)
        if state is None:
            node.get_logger().warn(
                f"attempt {attempt}: load_controller no response and "
                f"list_controllers also silent — retrying")
            continue
        if state == "":
            node.get_logger().warn(
                f"attempt {attempt}: controller {name} not in list yet "
                f"(load result={res!r}) — retrying")
            continue
        node.get_logger().info(
            f"{name} present on CM with state={state!r} (attempt {attempt})")
        return True
    node.get_logger().error(f"failed to load {name} after {MAX_ATTEMPTS} attempts")
    return False


def configure(node: Node, cfg_c, list_c, name: str) -> bool:
    for attempt in range(1, MAX_ATTEMPTS + 1):
        res = _call(node, cfg_c, ConfigureController.Request(name=name),
                    CALL_TIMEOUT_SEC)
        if res is not None and res.ok:
            node.get_logger().info(f"configured {name} (attempt {attempt})")
            return True
        state = _state_of(node, list_c, name, CALL_TIMEOUT_SEC)
        if state in ("inactive", "active"):
            node.get_logger().info(
                f"{name} already at state={state} (attempt {attempt})")
            return True
        node.get_logger().warn(
            f"attempt {attempt}: configure returned {res!r}, "
            f"list reports state={state!r} — retrying")
    node.get_logger().error(f"failed to configure {name} after {MAX_ATTEMPTS} attempts")
    return False


def activate(node: Node, switch_c, list_c, name: str) -> bool:
    for attempt in range(1, MAX_ATTEMPTS + 1):
        req = SwitchController.Request()
        req.activate_controllers = [name]
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        res = _call(node, switch_c, req, CALL_TIMEOUT_SEC)
        if res is not None and res.ok:
            node.get_logger().info(f"activated {name} (attempt {attempt})")
            return True
        state = _state_of(node, list_c, name, CALL_TIMEOUT_SEC)
        if state == "active":
            node.get_logger().info(
                f"{name} confirmed active via list (attempt {attempt})")
            return True
        node.get_logger().warn(
            f"attempt {attempt}: switch returned {res!r}, "
            f"list reports state={state!r} — retrying")
    node.get_logger().error(f"failed to activate {name} after {MAX_ATTEMPTS} attempts")
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("controller")
    ap.add_argument("--controller-manager", default="/controller_manager")
    ap.add_argument("--controller-manager-timeout", type=float, default=60.0)
    ap.add_argument("--inactive", action="store_true")
    args, _unknown = ap.parse_known_args()

    rclpy.init()
    n = Node("robust_controller_spawner")
    cm = args.controller_manager.rstrip("/")
    name = args.controller

    load_c = n.create_client(LoadController, f"{cm}/load_controller")
    cfg_c = n.create_client(ConfigureController, f"{cm}/configure_controller")
    switch_c = n.create_client(SwitchController, f"{cm}/switch_controller")
    list_c = n.create_client(ListControllers, f"{cm}/list_controllers")

    # Wait for services to appear — this is a discovery wait, not a call.
    for svc_name, c in [
        ("load_controller", load_c),
        ("configure_controller", cfg_c),
        ("switch_controller", switch_c),
        ("list_controllers", list_c),
    ]:
        if not _wait_service(n, c, args.controller_manager_timeout):
            n.get_logger().error(
                f"service {cm}/{svc_name} unavailable within "
                f"{args.controller_manager_timeout}s")
            return 1

    if not load(n, load_c, list_c, name):
        return 1
    if not configure(n, cfg_c, list_c, name):
        return 1
    if args.inactive:
        n.get_logger().info(f"leaving {name} inactive (--inactive)")
        return 0
    if not activate(n, switch_c, list_c, name):
        return 1
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except KeyboardInterrupt:
        code = 130
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)
