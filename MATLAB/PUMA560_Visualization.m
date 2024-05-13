%% PUMA 560 Robot Visualization
% This script demonstrates the forward and inverse kinematics of the PUMA 560 robot.


% Load the robot model
robot = loadrobot('puma560', 'DataFormat', 'struct', 'Gravity', [0 0 -9.81]);

%% Example 1: Forward Kinematics
% Define joint angles for the first example
joint_angles_example1 = [0, pi/4, pi/4, 0, pi/4, 0];

% Calculate the forward kinematics
pose_example1 = forwardKinematics(joint_angles_example1)


% Visualize the robot for Example 1
figure('Name', 'PUMA 560 - Forward Kinematics Example 1');
visualize(robot, joint_angles_example1);

%% Example 2: Inverse Kinematics
% Define the desired end-effector pose for the second example
% The position is closer to the base with a moderate elevation and a simple orientation
desired_pose_example2 = [0.4, 0, 0.5, 0, pi/6, 0];

% Extract the desired position and orientation from the array
Px = desired_pose_example2(1);
Py = desired_pose_example2(2);
Pz = desired_pose_example2(3);
phi = desired_pose_example2(4);
theta = desired_pose_example2(5);
psi = desired_pose_example2(6);

% Calculate the inverse kinematics
joint_angles_example2 = inverseKinematics(Px, Py, Pz, phi, theta, psi);

% Visualize the robot for Example 2
figure('Name', 'PUMA 560 - Inverse Kinematics Example 2');
visualize(robot, joint_angles_example2);
