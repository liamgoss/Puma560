function visualize(robot, desired_pose)
    % Load the robot model
    %robot = loadrobot('puma560', 'DataFormat', 'struct');
    
    % Calculate the joint angles from the inverse kinematics
    joint_angles = inverseKinematics(desired_pose(1), desired_pose(2), desired_pose(3), desired_pose(4), desired_pose(5), desired_pose(6));
    
    % Create a structure for the robot configuration
    config = homeConfiguration(robot);
    
    % Assign the joint angles to the configuration structure
    for i = 1:length(joint_angles)
        config(i).JointPosition = joint_angles(i);
    end
    
    % Show the robot configuration
    show(robot, config);
end
