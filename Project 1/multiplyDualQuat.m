% pq2DualQuat: Returns a dual quaternion for a given quaternion
%
% dual_quat = pq2DualQuat(pos, quat) Given a 1-by-4 quaternion in the form of [q_0 q_1 q_2
% q_3] and  the given function returns a rotaton matrix R
%
% output1 = 3-by-3 roation matrix R
% output2 = description of what the second output is/means include units if appropriate
%
% input1 =  1-by-4  quaternion with first term being the scalar
% input2 = description of what the second input is/means include units if appropriate 
%
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023
function dual_quat = multiplyDualQuat(dual_quat_left, dual_quat_right) 
    % Extract the rotation and translation parts of the left dual quaternion
    rot_left = dual_quat_left.rot;
    disp_left = dual_quat_left.disp;
    
    % Extract the rotation and translation parts of the right dual quaternion
    rot_right = dual_quat_right.rot;
    disp_right = dual_quat_right.disp;
    
    % Compute the dual quaternion product
    rot_result = multiplyQuat(rot_left, rot_right);
    disp_result = multiplyQuat(rot_left, disp_right) + multiplyQuat(disp_left, rot_right);
    
    % Combine the results into a new dual quaternion structure
    dual_quat.rot = rot_result;
    dual_quat.disp = disp_result;
end

