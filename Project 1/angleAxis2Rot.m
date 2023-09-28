% angleAxis2Rot: Returns the rotation matrix encoded by a rotation of theta radians
% about the unit vector k axis.
%
% R = angleAxis2Rot(k, theta) Provides us the 3-by-3 roation matrix for the  angle 
% axis represenation for a rotation of theta about k vector 
%
% output1 = 3-by-3 rotation matrix
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = Axis parameter i.e, k being 3-by-1 vector
% input2 = Angle parameter theta in radians 
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 1st 2023

function [R] = angleAxis2Rot(Omega)
  theta = norm(Omega);
    if theta < eps % Check if theta is very close to zero
        R = eye(3); % Return identity matrix for no rotation
    else
    k = Omega/theta;
    k_x = [0 -k(3) k(2);
        k(3) 0 -k(1);
        -k(2) k(1) 0 ];
    R = cos(theta)*eye(3) + (1-cos(theta))*k*transpose(k)+sin(theta)*k_x;
    end 
end 
