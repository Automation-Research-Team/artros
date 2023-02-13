traj = right_arm.plan()
new_traj = RobotTrajectory()
new_traj.joint_trajectory = traj.joint_trajectory
n_joints = len(traj.joint_trajectory.joint_names)
n_points = len(traj.joint_trajectory.points)

spd = 2.0

for i in range(n_points):
    traj.joint_trajectory.points[i].time_from_start =
    traj.joint_trajectory.points[i].time_from_start / spd
for j in range(n_joints):
    new_traj.joint_trajectory.points[i].velocities[j] =
    traj.joint_trajectory.points[i].velocities[j] * spd
    new_traj.joint_trajectory.points[i].accelerations[j] =
    traj.joint_trajectory.points[i].accelerations[j] * spd
    new_traj.joint_trajectory.points[i].positions[j] =
    traj.joint_trajectory.points[i].positions[j]

self.right_arm.execute(new_traj)
