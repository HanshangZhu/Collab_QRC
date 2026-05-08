"""Domain-specific launch module builders for go2_gazebo_sim."""

import os


def _find_mujoco_plugin_dir() -> str:
    """Locate the MuJoCo `plugin/` dir containing libobj_decoder.so, libstl_decoder.so, etc.

    The ros2 launch parser runs under /usr/bin/python3 (ros2's shebang) which
    may not have mujoco installed, so we cannot `import mujoco` here. Instead
    we probe the conda env (CONDA_PREFIX) and user-site locations directly.
    Override with env var MUJOCO_PLUGIN_DIR if neither is correct.
    """
    override = os.environ.get("MUJOCO_PLUGIN_DIR")
    if override and os.path.isdir(override):
        return override
    candidates = []
    conda = os.environ.get("CONDA_PREFIX")
    if conda:
        candidates.append(os.path.join(conda, "lib", "python3.10",
                                       "site-packages", "mujoco", "plugin"))
    candidates.append(os.path.join(os.path.expanduser("~"),
                                   ".local", "lib", "python3.10",
                                   "site-packages", "mujoco", "plugin"))
    for c in candidates:
        if os.path.isdir(c):
            return c
    raise RuntimeError(
        "MuJoCo plugin dir not found. Tried: "
        + ", ".join(candidates)
        + ". Set MUJOCO_PLUGIN_DIR to override."
    )
