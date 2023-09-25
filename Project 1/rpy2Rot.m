% rpy2Rot : Returns a rotation matrix for roll, pitch and yaw end-effector
% 
% R = rpy2Rot(roll, pitch, yaw) Returns a rotation matrix describing a rotation about 
% the X axis (roll in radians), Y axis (pitch in radians) and Z axis (yaw in radians).
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
function[R] = rpy2Rot(roll, pitch, yaw)

R = rotZ(yaw)*rotY(pitch)*rotX(roll);

end