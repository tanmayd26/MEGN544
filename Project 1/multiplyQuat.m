% multiplyQuat: Returns a quaternion product
%
% Q = multiplyQuat(Q_left, Q_right) Given a 4-by-1 quaternion in the form of [q_0; q_vec] 
% and  the given function returns a quaternion.
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
function Q = multiplyQuat(Q_left, Q_right)
   
   % Extract the components of the left quaternion
    q0_left = Q_left(1);
    q_vec_left = Q_left(2:4);
    
    % Extract the components of the right quaternion
    q0_right = Q_right(1);
    q_vec_right = Q_right(2:4);
    
    % Compute the quaternion product
    q0_result = q0_left * q0_right - dot(q_vec_left, q_vec_right);
    q_vec_result = q0_left * q_vec_right + q0_right * q_vec_left + cross(q_vec_left, q_vec_right);
    
    % Combine the results into a 4x1 vector
    Q = [q0_result; q_vec_result];
end