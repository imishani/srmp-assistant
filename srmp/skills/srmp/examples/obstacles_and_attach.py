"""Add obstacles relative to the robot, attach a grasped object, plan, then detach.

Object positions come from forward kinematics, never from guesses.
"""
import numpy as np

import srmp

planner = srmp.PlannerInterface()
planner.add_robot("panda")

start = np.radians([0, -45, 0, -135, 0, 90, 45])
planner.set_qpos("panda", start)
ee = planner.compute_fk("panda", start)  # world-frame pose of panda_hand

# A table 2 cm thick, its top 25 cm below the hand, in front of the robot base.
# Identity orientation is [1, 0, 0, 0].
table = srmp.Pose(np.array([ee.p[0] + 0.15, 0.0, ee.p[2] - 0.25 - 0.01]), np.array([1.0, 0.0, 0.0, 0.0]))
planner.add_box("table", [0.4, 0.8, 0.02], table, color=(0.6, 0.45, 0.3))

# A wall to one side of the workspace.
wall = srmp.Pose(np.array([ee.p[0], -0.35, ee.p[2]]), np.array([1.0, 0.0, 0.0, 0.0]))
planner.add_box("wall", [0.6, 0.02, 0.6], wall, color="#888888")

# A 4 cm cube between the fingers (10 cm below the hand frame). It touches the fingers, so
# the state counts as colliding until the cube is attached; attaching allows those contacts.
cube = srmp.Pose(ee.p + np.array([0.0, 0.0, -0.10]), np.array([1.0, 0.0, 0.0, 0.0]))
planner.add_box("cube", [0.04, 0.04, 0.04], cube, color=(0.85, 0.2, 0.2))
planner.attach_object("cube", "panda", planner.get_link_index("panda", "panda_hand"))
assert planner.is_object_attached("cube")
assert not planner.is_state_colliding("panda"), "start state is in collision"

# Move the held cube to the other side, keeping the hand orientation.
target = srmp.Pose(ee.p + np.array([0.0, 0.30, 0.0]), ee.q.copy())
planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "50", "time_limit": "10"})
traj = planner.plan(start, srmp.GoalConstraint(srmp.GoalType.POSE, [target]))
if len(traj.positions) == 0:
    raise SystemExit("no plan found")
print(f"{len(traj.positions)} waypoints; objects: {planner.get_object_names()}")

# Release the cube at the goal. set_qpos does not move attached bodies: call
# update_attached_bodies_pose() before detaching, or the cube is left at its old pose.
planner.set_qpos("panda", np.asarray(traj.positions[-1]))
planner.update_attached_bodies_pose()
planner.detach_object("cube")  # also_remove=True would delete it from the scene instead
print("cube attached after detach:", planner.is_object_attached("cube"))
assert len(traj.positions) > 0
