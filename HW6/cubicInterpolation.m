function coefficients = cubicInterpolation(initialPose, finalPose, T,a,b)
    % Compute the joint angles at the initial and final poses using inverse kinematics
    q_initial = inversekinematics(initialPose,a,b);
    q_final = inversekinematics(finalPose,a,b);
    
    % Compute the time vector for the interpolation
    t = 0:1:T; % You can adjust the time step (0.01) as needed
    
    % Compute the joint angles at each time step using cubic interpolation
    q_interp = zeros(length(t), length(q_initial));
    for i = 1:length(q_initial)
        q_interp(:, i) = cubicSplineInterpolation(q_initial(i), q_final(i), T, t);
        disp(q_interp(:,i));
    end
    
    % Compute the end-effector positions at each time step using forward kinematics
    endEffectorPositions = zeros(4, length(t));
    for i = 1:length(t)
        endEffectorPositions(:, i) = forwardkinematics(q_interp(i, :),a,b);
    end
    
    % Create the coefficients matrix for the cubic interpolation
    coefficients = zeros(4, length(q_initial), length(t) - 1);
    for i = 1:length(q_initial)
        for j = 1:length(t) - 1
            %dt = t(j + 1) - t(j);
            coefficients(:, i, j) = [q_interp(j, i); 0; (3 / (T^2)) * (q_interp(j + 1, i) - q_interp(j, i)); (2 / (T^3)) * (q_interp(j, i) - q_interp(j + 1, i))];
        end
    end
end


