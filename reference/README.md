# reference/ — app-layer code, not built

These files are **not part of any ROS2 package** and are not compiled or installed.
They are parked here on the way to `bonicOS-robot-app`.

| File | What it was |
|---|---|
| `robot_manager.py` | Orchestrator: SLAM/Nav2 process lifecycle, map save, named locations, patrol, precise-move queue, per-detector vision toggles |
| `robot_agent.py` | Gemini ReAct agent — a pure client of `robot_manager.py`'s services |
| `agent_config.yaml` | `robot_agent.py`'s config (model, arm IK lengths, limits) |

## Why they are here and not deleted

Orchestration belongs to `bonicOS-robot-app`, which is shared with M1 and already
carries a `ROBOT_SERIES=A` config. M1 did the same thing with its own
`robot_manager.py`.

But robot_app does **not** yet implement everything these provided. No equivalent was
found for:

- the precise-move queue
- patrol / waypoint following
- the named-location store
- per-detector vision toggles — `robot_manager.py` was the only publisher of
  `/vision/control`, which `vision_pipeline.py` still subscribes to for its enable
  flags. With nothing publishing it, the pipeline runs but every detector stays off.

So deleting them now would be a silent capability regression. Move each one only once
robot_app covers it.

## Running them anyway (transitional)

They are plain rclpy nodes. Nothing installs them, so run directly:

    python3 reference/robot_manager.py

Note their service/topic surface assumes the pre-restructure launch layout — in
particular `robot_manager.py` shells out to `nav2_bringup`'s and `slam_toolbox`'s own
launch files, not this repo's new `bonicbot_a2_nav` ones.
