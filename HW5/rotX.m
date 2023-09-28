% rotX : Returns a rotation matrix for rotation about X axis 
% 
% R = rotX(theta) Returns a rotation matrix describing a rotation about 
% the X axis (theta in radians).
% 
% output1 = 3-by-3 rotation matrix 
% output2 = description of what the second output is/means include units if appropriate 
% 
% input1 = theta represent a rotation about X in radians
% input2 = description of what the second input is/means include units if appropriate
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 1st 2023
function [R] = rotX(theta)

R = [1 0 0;
    0 cos(theta) -sin(theta);
    0 sin(theta) cos(theta)];


end