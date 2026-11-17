---
name: srmp
description: >-
  Write correct Python code for SRMP (search-based robot motion planning; `import srmp`,
  `srmp.PlannerInterface`). Use when the user asks for motion planning for robot arms and
  manipulators with SRMP: loading robots from the SRMP robot registry (`add_robot("panda")`),
  building scenes (boxes, spheres, cylinders, meshes, point clouds), attaching grasped
  objects, forward/inverse kinematics, planning to joint or end-effector pose goals with
  `make_planner` and `plan`, choosing planners (wAstar, ARAstar, MHAstar, MGS, vamp_rrtc) and
  planner_context settings, the VAMP collision backend and `srmp vamp` plugins, multi-robot
  planning with ECBS / xECBS and `plan_multi`, and the srmp MCP tools (`run_srmp_code`).
  Also use when the code imports srmp or mentions PlannerInterface, GoalConstraint, Pose or
  Trajectory from srmp.
metadata:
  srmp_version: "0.2.2"
---

# SRMP motion planning

SRMP plans collision-free motions for robot manipulators with search-based planners.
Everything goes through one `srmp.PlannerInterface` object: load robots, build the scene,
make a planner, plan, check the result.

If anything here conflicts with the installed `srmp` stubs (`python -c 'import srmp, os; print(os.path.dirname(srmp.__file__))'`/`__init__.pyi`, `planner_interface.pyi`), trust the stubs.

## Core workflow

```python
import numpy as np
import srmp

planner = srmp.PlannerInterface()
planner.add_robot("panda")                 # 1. robot from the registry (downloads on first use)

start = np.radians([0, -45, 0, -135, 0, 90, 45])
planner.set_qpos("panda", start)
ee = planner.compute_fk("panda", start)    # 2. scene: place objects relative to real poses
table = srmp.Pose(np.array([ee.p[0] + 0.15, 0.0, ee.p[2] - 0.26]), np.array([1.0, 0.0, 0.0, 0.0]))
planner.add_box("table", [0.4, 0.8, 0.02], table)

goal = srmp.GoalConstraint(srmp.GoalType.JOINTS,  # 3. goal: a LIST of goals
                           [start + np.radians([40, 0, 0, 0, 0, 0, 0])])
planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "50",  # 4. string values
                                 "time_limit": "10"})
traj = planner.plan(start, goal)           # 5. plan

if len(traj.positions) == 0:               # 6. empty trajectory = no plan found
    raise RuntimeError("no plan found")
print(len(traj.positions), "waypoints; last:", np.round(traj.positions[-1], 3))
```

- Pose goal: `srmp.GoalConstraint(srmp.GoalType.POSE, [pose])` where `pose` is an
  `srmp.Pose(p, q)` for the end effector in the world frame. See
  [examples/pose_goal.py](examples/pose_goal.py).
- `plan()` does not raise when planning fails; it returns an empty trajectory.
  `plan_multi()` returns an empty dict. A colliding start is a common cause: check
  `planner.is_state_colliding()` with the robot at the start configuration.
- `traj.positions` is a list of waypoints in move-group joint order
  (`planner.get_move_group_joint_names(name)`); `traj.times` gives each waypoint's time.

## Rules

<!-- rules:start -->
- Use `planner.add_robot("name")` to load robots (auto-downloads). Never invent file paths.
- **Never guess 3D positions.** Always query FK/link poses first, then compute offsets.
- `np.radians()` for degrees. Normalize quaternions. GoalConstraint data is `[goal_array]` (2-D).
- All planner_context values are **strings**: `{"weight": "10."}` not `{"weight": 10.0}`.
- Collision backend: coal by default. `"validity_backend": "auto"` uses the much faster VAMP
  backend when `planner.is_robot_vamp_supported(name)` is True, else coal. VAMP ignores gripper
  finger positions (fingers are fixed in its model), so keep coal when fingers close on or near objects.
- xECBS/ECBS take joint-space goals only. Per-robot `heuristic_<name>` and `mprim_path_<name>`
  keys are optional; the defaults are the time-aware `bfs` and the built-in timed primitives.
- `compute_fk` returns, and `compute_ik` / `collision_aware_ik` take, poses in the WORLD frame.
- Planners: e.g. `wAstar`, `ARAstar`, `MHAstar`, `Astar` | Multi: `xECBS`, `ECBS`; see references/planner-settings.md for all.
- Heuristics: e.g. `bfs`, `joint_euclidean`, `joint_euclidean_remove_time`; see references/planner-settings.md for all.
- **Verify before you report.** Before saying a task is done, re-read the scene in code
  (`planner.get_object_names()`, the poses you changed, `planner.get_qpos(...)`) and report
  what you checked. Never say something is "unchanged" without reading it back.
- `Pose.q` is `[w, x, y, z]` (scalar first); identity is `[1, 0, 0, 0]`.
- `plan()` returns a Trajectory; empty `traj.positions` means no plan was found — check it before using the result.
<!-- rules:end -->

More facts that trip people up:

- Plan to `GoalType.JOINTS` or `GoalType.POSE` with exactly one goal in the list. The other
  `GoalType` members (`POSITION`, `MULTI_GOAL_*`) raise in srmp 0.2.1–0.2.2.
- The one exception to "values are strings": `inadmissible_heuristics` (MHAstar) also takes
  a list of strings. A float or int value makes `make_planner` raise `TypeError`.
- `collision_aware_ik(name, pose, q_seed)` returns `(q, status)` with status `"found"`,
  `"unreachable"` or `"blocked"` (`q` is `None` unless found), and leaves the robot at `q`
  on success. `compute_ik` returns `(success, q)`.
- `set_qpos` does not move attached objects. Call `planner.update_attached_bodies_pose()`
  before `detach_object`, or the object is released at its old pose.
- An object touching the gripper makes the state colliding until it is attached with
  `attach_object`, which allows those contacts.
- Two copies of one registry robot need distinct names: `add_robot("panda", name="panda0")`.
  Move a robot's base with `set_base_pose(name, pose)`.
- `vamp_rrtc` plans only with the VAMP backend (`"validity_backend": "vamp"` or `"auto"`)
  and only to joint goals; otherwise it returns an empty trajectory.
- Unknown `planner_id`, heuristic or `validity_backend` names raise `RuntimeError` in
  `make_planner`. List this build's planners with `planner.print_available_planners()`.

## Read next

| When you need | Read |
|---|---|
| Exact signatures of every class and method | [references/api.md](references/api.md) |
| Every `make_planner` key, per planner, with defaults | [references/planner-settings.md](references/planner-settings.md) |
| Which planner and heuristic to use | [references/choosing-a-planner.md](references/choosing-a-planner.md) |
| Several robots at once (ECBS / xECBS, `plan_multi`) | [references/multi-robot.md](references/multi-robot.md) |
| VAMP backend, supported robots, `srmp vamp` plugins | [references/vamp.md](references/vamp.md) |
| Using the srmp MCP tools | [references/mcp-tools.md](references/mcp-tools.md) |
| Joint goal, full script | [examples/joint_goal.py](examples/joint_goal.py) |
| Pose goal from forward kinematics, with IK check | [examples/pose_goal.py](examples/pose_goal.py) |
| Obstacles, attach, plan, detach | [examples/obstacles_and_attach.py](examples/obstacles_and_attach.py) |
| Two arms with xECBS | [examples/multi_robot.py](examples/multi_robot.py) |

Robot registry: `srmp.robots.list_available()` returns a dict of robot-name lists,
`{"remote": [...], "local": [...], "custom": [...]}` (`local` = already downloaded);
`srmp.robots.get(name)` gives `.end_effector`, `.default_qpos`, `.joint_names` and more.

## MCP tools

If tools named `run_srmp_code`, `reset_session`, `describe_scene` and `start_visualizer`
(the `srmp` MCP server) are available, use them: run SRMP code through `run_srmp_code`
instead of writing throwaway scripts, and check the scene with `describe_scene` before you
report. State (`planner`, robots, objects, variables) persists between calls. See
[references/mcp-tools.md](references/mcp-tools.md).
