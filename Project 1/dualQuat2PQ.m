% dualQuat2PQ: Returns displacement and 
%
% [pos,quat] = dualQuat2PQ(dual_quat Given a 1-by-4 quaternion in the form of [q_0 q_1 q_2
% q_3] and  the given function returns a rotaton matrix R
%
% output1 = 
% output2 = 
%
% input1 =  1-by-4  quaternion with first term being the scalar
% input2 = description of what the second input is/means include units if appropriate 
%
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023
function [pos,quat] = dualQuat2PQ(dual_quat)

% Extract the rotation and translation parts from the dual quaternion
    quat = dual_quat.rot;
    disp_quaternion = dual_quat.disp;

    conjugate = [quat(1);-quat(2);-quat(3);-quat(4)];
% Return the position and quaternion
    pos = 2*multiplyQuat(disp_quaternion,conjugate);
    pos = pos(2:4);
  

end
