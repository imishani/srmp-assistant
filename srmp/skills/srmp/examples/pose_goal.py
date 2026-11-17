"""Plan the panda end effector to a Cartesian pose derived from forward kinematics.

Never guess 3D positions: read a real pose with compute_fk / get_link_pose and offset it.
"""
import numpy as np

import srmp

planner = srmp.PlannerInterface()
planner.add_robot("panda")

start = np.radians([0, -45, 0, -135, 0, 90, 45])
planner.set_qpos("panda", start)

# compute_fk returns the end-effector pose (panda_hand for the panda) in the WORLD frame.
ee = planner.compute_fk("panda", start)
print("start EE position:", np.round(ee.p, 3), "orientation [w, x, y, z]:", np.round(ee.q, 3))

# Target: 20 cm forward and 10 cm lower, same orientation (Pose.q is [w, x, y, z]).
target = srmp.Pose()
target.p = ee.p + np.array([0.20, 0.0, -0.10])
target.q = ee.q.copy()

# Optional: check the pose is reachable and collision-free before planning.
# collision_aware_ik returns (q, status) with status "found", "unreachable" or "blocked".
q_ik, status = planner.collision_aware_ik("panda", target, list(start))
print("IK status:", status)
planner.set_qpos("panda", start)  # collision_aware_ik moves the robot to q_ik on success

goal = srmp.GoalConstraint(srmp.GoalType.POSE, [target])
planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "50", "time_limit": "10"})
traj = planner.plan(start, goal)
if len(traj.positions) == 0:
    raise SystemExit("no plan found")

# Verify: where did the end effector end up?
reached = planner.compute_fk("panda", np.asarray(traj.positions[-1]))
print("position error (m):", np.linalg.norm(reached.p - target.p))
assert len(traj.positions) > 0
