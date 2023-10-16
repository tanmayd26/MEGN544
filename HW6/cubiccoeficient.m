% cubiccoeficient: Returns the cubic coefficient
%
% [a0, a1, a2, a3] = cubiccoeficient(initial_pose, final_pose, T) Gives us the poses depending on
% parameters.
%
% output1 = Parameter 1
% output2 = Parameter 2
%
% input1 = initial transformation matrix
% input2 = final transformation matrix
% input 3 = final time.
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 3rd 2023
function [a0, a1, a2, a3] = cubiccoeficient(initial_pose, final_pose, T)
     % Compute the joint angles at the initial and final poses using inverse kinematics

        a0 = initial_pose;
        a1 = 0;
        a2 = (3/T^2)*(final_pose-initial_pose);
        a3 = (-2/T^3)*(final_pose-initial_pose);

end