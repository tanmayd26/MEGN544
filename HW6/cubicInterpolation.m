function coefficients = cubicInterpolation(initial_pose, final_pose, T,inverseKinematics,forwardKinematics)
     % Compute the joint angles at the initial and final poses using inverse kinematics
  
        [theta1_init, theta2_init, d3_init, theta4_init, theta5_init, theta6_init] = inverseKinematics(initial_pose);
       [theta1_final, theta2_final, d3_final, theta4_final, theta5_final, theta6_final]= inverseKinematics(final_pose);
  
    % Create the time vector for the interpolation
    t = 0:0.01:T; % You can adjust the time step (0.01) as needed
    
    
    trans = forwardKinematics(theta1_init,theta2_init,d3_init,theta4_init,theta5_init,theta6_init);

    % Compute the end-effector poses at each time step using forward kinematics
   T = zeros(4, 4);
    for i = 1:length(t)
        t_factor = t(i);
        inter_pose = (1 - t_factor)* initial_pose + t_factor*final_pose;
        [theta1, theta2, d3, theta4, theta5, theta6] = inverseKinematics(inter_pose);
        T(i) = trans*forwardKinematics(theta1,theta2,d3,theta4,theta5,theta6);
    end
    
    % Create the coefficients matrix for the cubic interpolation
    coefficients = zeros(4, 4, length(q_initial), length(t) - 1);
    for i = 1:length(q_initial)
        for j = 1:length(t) - 1 
            dt = t(j + 1) - t(j);
            q_curr = q_interp(j, i);
            q_next = q_interp(j + 1, i);
            coefficients(:, :, i, j) = interpolateTransformationMatrix(initial_pose, final_pose, q_curr, q_next, dt);
        end
    end
end