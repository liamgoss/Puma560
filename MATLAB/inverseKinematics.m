function joint_angles = inverseKinematics(Px, Py, Pz, phi, theta, psi)
    % PUMA 560 constants (link lengths, etc.)
    d1 = 0.67183;   % Distance along Z1 from frame 0 to frame 1
    a2 = 0.4318;    % Distance along X2 from frame 1 to frame 2
    a3 = -0.02032;  % Distance along X3 from frame 2 to frame 3
    d4 = 0.4318;    % Distance along Z4 from frame 3 to frame 4
    d6 = 0.05650;   % Distance along Z6 from frame 5 to frame 6 (end-effector)
    
    % Compute theta1
    theta1 = atan2(Py, Px);
    
    % Calculate r and s for the wrist center, considering orientation
    r = sqrt(Px^2 + Py^2) - a3; % Horizontal distance from base to wrist, minus offset
    s = Pz - d4;                % Vertical distance from base to wrist, minus link 4 length
    
    % Use the Cosine Law to find theta3
    D = (r^2 + s^2 - a2^2 - d4^2) / (2 * a2 * d4);
    % Check if D is within the valid range for acos
    if D < -1 || D > 1

       % Right now the position given isn't feasible
       % We can either error the code and prevent it from moving
       % OR
       % try to move it to the next nearest position that *is* feasible

        % error('The point is outside the reachable workspace of the robot.');
        disp('Warning: The desired pose is outside the reachable workspace. Adjusting to nearest reachable pose.');
        D = max(min(D, 1), -1);  % Clamp D to the range [-1, 1]
    end
    
    % Now it's safe to calculate theta3
    theta3 = atan2(sqrt(1 - D^2), D);
    
    % Compute theta2
    theta2 = atan2(s, r) - atan2(d4 * sin(theta3), a2 + d4 * cos(theta3));
    
    % Calculate the orientation angles theta4, theta5, and theta6
    % First, compute the rotation matrix from the base to the 3rd joint
    R0_3 = [cos(theta1)*cos(theta2 + theta3), -sin(theta1),  cos(theta1)*sin(theta2 + theta3);
            sin(theta1)*cos(theta2 + theta3),  cos(theta1),  sin(theta1)*sin(theta2 + theta3);
            -sin(theta2 + theta3),             0,            cos(theta2 + theta3)];
    
    % Desired rotation matrix for end-effector
    % XYZ based euler angles, not ZYX or ZYZ, etc
    R_des = eul2rotm([phi, theta, psi], 'XYZ'); % Euler angles to rotation matrix
    
    % Compute the wrist rotation matrix
    R3_6 = R0_3.' * R_des; % Transpose of R0_3 times desired end-effector rotation
    
    % Extract Euler angles from R3_6, representing theta4, theta5, and theta6
    theta4 = atan2(R3_6(2,3), R3_6(1,3));
    theta5 = atan2(sqrt(R3_6(1,3)^2 + R3_6(2,3)^2), R3_6(3,3));
    theta6 = atan2(R3_6(3,2), -R3_6(3,1));
    
    joint_angles = [theta1 theta2 theta3 theta4 theta5 theta6];
    % Display results
    disp('The calculated joint angles are:');
    disp(['Theta1: ', num2str(theta1)]);
    disp(['Theta2: ', num2str(theta2)]);
    disp(['Theta3: ', num2str(theta3)]);
    disp(['Theta4: ', num2str(theta4)]);
    disp(['Theta5: ', num2str(theta5)]);
    disp(['Theta6: ', num2str(theta6)]);

end