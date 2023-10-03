function [a0, a1, a2, a3] = cubiccoeficient(initial_pose, final_pose, T)
     % Compute the joint angles at the initial and final poses using inverse kinematics

        a0 = initial_pose;
        a1 = 0;
        a2 = (3/T^2)*(final_pose-initial_pose);
        a3 = (-2/T^3)*(final_pose-initial_pose);

end