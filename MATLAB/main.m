% Define desired end-effector poses [Px, Py, Pz, phi, theta, psi]
desired_poses = [
    0.3, 0, 0.2, 0, 0, 0;      % Initial neutral position
    0.3, 0, 0.5, 0, pi/4, 0;   % Raise and extend arm
    0.3, 0, 0.2, 0, -pi/4, 0;  % Lower and retract arm
    0.3, 0, 0.5, 0, pi/4, 0;   % Raise and extend arm again
];

% Initialize matrix to store calculated joint angles
jointAngles = zeros(size(desired_poses, 1), 6);

% Calculate joint angles for each pose using the inverse kinematics function
for i = 1:size(desired_poses, 1)
    [Px, Py, Pz, phi, theta, psi] = desired_poses(i, :);
    jointAngles(i, :) = inverseKinematics(Px, Py, Pz, phi, theta, psi);
end

% Verify the positions using forward kinematics
for i = 1:size(jointAngles, 1)
    T = forwardKinematics(jointAngles(i, :));
    disp('Calculated End-Effector Position:');
    disp(T(1:3, 4)');  % Display the translational part of the transformation matrix
end

% Number of steps in the animation between poses
numSteps = 20;
% Initialize the sequence matrix
jointAnglesSequence = [];

% Generate interpolated joint angles between each consecutive pair of poses
for i = 1:size(jointAngles, 1) - 1
    startAngles = jointAngles(i, :);
    endAngles = jointAngles(i + 1, :);
    for j = 0:numSteps
        interpolatedAngles = (1 - j/numSteps) * startAngles + (j/numSteps) * endAngles;
        jointAnglesSequence = [jointAnglesSequence; interpolatedAngles];
    end
end

% Call the animation function
animateRobot(robot, jointAnglesSequence);
