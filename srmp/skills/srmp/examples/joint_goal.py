"""Plan a panda arm from one joint configuration to another (joint-space goal)."""
import numpy as np

import srmp

planner = srmp.PlannerInterface()
planner.add_robot("panda")  # registry name; downloads and caches the robot on first use

# Joint values are radians, in move-group order (planner.get_move_group_joint_names("panda")).
start = np.radians([50, 47, -10, -35, -22, 93, 39])
goal_q = np.radians([21, 29, -30, -104, -162, 52, -118])
planner.set_qpos("panda", start)

# GoalConstraint data is a list of goals: [goal_q], not goal_q.
goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_q])

# Every planner_context value is a string.
planner.make_planner(["panda"], {"planner_id": "wAstar", "heuristic": "bfs",
                                 "weight": "50", "time_limit": "10"})
traj = planner.plan(start, goal)

# An empty trajectory means no plan was found (plan() does not raise for that).
if len(traj.positions) == 0:
    raise SystemExit("no plan found")
print(f"{len(traj.positions)} waypoints, duration {traj.times[-1]:.2f} s")
print("final configuration (deg):", np.round(np.degrees(traj.positions[-1]), 1))
assert len(traj.positions) > 0
