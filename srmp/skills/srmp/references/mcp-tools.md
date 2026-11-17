# The srmp MCP tools

The `srmp` MCP server runs SRMP code in one persistent Python worker, so an assistant can
build a scene step by step, plan, and inspect the result without writing throwaway scripts.

## Setup

- Install: `pip install "srmp[mcp]"` (the MCP server needs Python 3.10 or newer).
- Start command (stdio): `srmp mcp`. Register it with your assistant, for example
  `claude mcp add srmp -- srmp mcp` for Claude Code.

If the tools below are not listed in your session, the server is not connected: write
normal Python scripts instead and run them with the user's Python.

## Tools

| Tool | Arguments | What it does |
|---|---|---|
| `run_srmp_code` | `code: str`, `timeout_s: float = 60` | Runs Python in the worker. `srmp`, `np` and `planner` (a `srmp.PlannerInterface`) are predefined. Returns the sections `stdout:`, `stderr:` and `error:` (empty ones omitted; `(no output)` when all are empty), each cut to 20,000 characters and ending in `... [truncated N chars]` when cut. An exception comes back in `error:`, it is not raised. |
| `reset_session` | none | Restarts the worker: a fresh `planner`, an empty scene, no variables. |
| `describe_scene` | none | JSON with `srmp_version`, `robots` (`name`, `dof`, `qpos`, `vamp_supported`), `objects` (with `attached`), and `validity_backends`. |
| `start_visualizer` | `port: int \| None` | Starts the Viser viewer on the worker's `planner`, reachable from this machine only (it binds 127.0.0.1), and returns its URL. Calling it again returns the same URL (and ignores `port`) until the session is reset or lost. |

The reference files are also served as MCP resources: `srmp://docs/SKILL.md` and
`srmp://docs/<file>` for each file in `references/`.

## How to use them

- State persists across `run_srmp_code` calls: robots, objects and variables you define stay
  until `reset_session` or a lost session. Build the scene once, then plan repeatedly.
- Do not create a second `srmp.PlannerInterface()`; use the predefined `planner` so
  `describe_scene` and the viewer see your scene.
- Keep each call short. A call that runs longer than `timeout_s` kills the worker. The result
  then starts with `SESSION LOST`: robots, objects and variables are gone and the scene
  must be rebuilt before continuing.
- Print summaries, not whole trajectories (`len(traj.positions)`, the last waypoint), since
  long output is truncated.
- Before you report a result, call `describe_scene` (or read the values back in code) and
  say what you checked.

Example `code` for `run_srmp_code`:

```python
planner.add_robot("panda")
start = np.radians([0, -45, 0, -135, 0, 90, 45])
goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [start + np.radians([40, 0, 0, 0, 0, 0, 0])])
planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "50"})
traj = planner.plan(start, goal)
print("waypoints:", len(traj.positions))
```

After `start_visualizer`, animate a result in a later `run_srmp_code` call:

```python
# skill-test: skip (needs the viewer started by the start_visualizer tool)
planner.get_visualizer().animate_trajectory(traj)                 # one robot
planner.get_visualizer().animate_trajectory(trajs)                # dict from plan_multi
```
