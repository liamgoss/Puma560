function T = forwardKinematics(joint_angles)
    % Unpack the joint angles
    theta1 = joint_angles(1);
    theta2 = joint_angles(2);
    theta3 = joint_angles(3);
    theta4 = joint_angles(4);
    theta5 = joint_angles(5);
    theta6 = joint_angles(6);

    % https://hive.blog/hive-196387/@juecoree/forward-kinematics-of-puma-560-robot-using-dh-method
    % The above link is where we got the params defined below
    
    % Define the link offsets and lengths based on DH parameters
    d1 = 0.67183;   % Distance along Z1 from frame 0 to frame 1
    a2 = 0.4318;    % Distance along X2 from frame 1 to frame 2
    a3 = -0.02032;  % Distance along X3 from frame 2 to frame 3 (Note: The negative sign indicates the direction opposite to the X3 axis)
    d4 = 0.4318;    % Distance along Z4 from frame 3 to frame 4
    d6 = 0.05650;   % Distance along Z6 from frame 5 to frame 6 (end-effector)

    % DH Parameters for PUMA 560
    % [theta, d, a, alpha]
    DH_params = [
        theta1, d1, 0, -pi/2;
        theta2, 0, a2, 0;
        theta3, 0, a3, -pi/2;
        theta4, d4, 0, pi/2;
        theta5, 0, 0, -pi/2;
        theta6, d6, 0, 0;
    ];
    
    % Number of joints
    n = size(DH_params, 1);
    
    % Initialize transformation matrix from base to end-effector
    T = eye(4);
    
    % Loop through each joint and compute the transformation matrix
    for i = 1:n
        theta = DH_params(i, 1);
        d = DH_params(i, 2);
        a = DH_params(i, 3);
        alpha = DH_params(i, 4);
        
        % Denavit-Hartenberg transformation matrix
        Ti = [
            cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,           sin(alpha),             cos(alpha),            d;
            0,           0,                      0,                     1;
        ];
        
        % Update overall transformation matrix
        T = T * Ti;
    end
end