% pq2DualQuat: Returns a dual quaternion for a given quaternion
%
% dual_quat = pq2DualQuat(pos, quat) Given a 4-by-1 quaternion in the form of [q_0 q_1 q_2
% q_3] and the 3-by-1 displacement vector the given function returns a 1-by-1 dual
% quaternion
%
% output1 = 4-by-1 dual quaternion
% output2 = description of what the second output is/means include units if appropriate
%
% input1 =  4-by-1  quaternion with first term being the scalar
% input2 = 3-by-1 displacement vector.
%
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023
function dual_quat = pq2DualQuat(pos, quat)
   %Get the translation part of dual quaternion
    disp = [0; pos(1); pos(2); pos(3)];

    % Normalize the real part of the dual quaternion
    quat = quat / norm(quat);

    % Build the dual quaternion by combining the real and dual parts
    dual_quat.rot = quat;
    dual_quat.disp = (1/2)*multiplyQuat(disp, quat);

end
