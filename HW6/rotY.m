% rotY : Returns a rotation matrix for rotation about Y axis 
% 
% R = rotY(theta) Returns a rotation matrix describing a rotation about 
% the Y axis (theta in radians).
% 
% output1 = 3-by-3 rotation matrix 
% output2 = description of what the second output is/means include units if appropriate 
% 
% input1 = theta represent a rotation about Y in radians
% input2 = description of what the second input is/means include units if appropriate
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 1st 2023

function[R] = rotY(theta)

R = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

end