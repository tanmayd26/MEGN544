% dhTransform: Returns a transformation matrix for given dh parameters.
%
% H = dhTransform(a, d, alpha, theta) Given all distances and angles the
% given function returns transformation matrix.
%
% output1 = 4-by-4 transformation matrix 
% output2 = description of what the second output is/means include units if appropriate
%
% input1 =  Distance along X_i axis
% input2 = Distance along Z_(i-1) axis
% input3 = Angle about X_i axis
% input4 = Angle about Z_(i-1) axis
%
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023

function H = dhTransform(a, d, alpha, theta)

H = [cos(theta), -sin(theta).*cos(alpha), sin(theta).*sin(alpha), a.*cos(theta);
     sin(theta), cos(theta).*cos(alpha), -cos(theta).*sin(alpha), a.*sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1];

end
