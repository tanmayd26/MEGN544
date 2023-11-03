% transError returns a 6x1 vector, where the first 3
% elements are position error (desired - current),
% and the last three elements are an angle-axis
% representation of rotation error. Both
% expressed in the shared base frame.
% Td is the homogenious matrix describing the
% desired coordinate pose (of the robot for
% example) in the reference frame (of the world
% for example).
% Tc is the homogenious matrix describing the
% current coordinate frame (of the robot for
% example) in the reference frame (of the world
% for example).
% error_vector is a 6x1 vector describing the error
% ([pos_error;rot_error]) as expressed in the
% shared base frame.
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023
function [error_vector] = transError(Td, Tc)

Rd(1:3,1:3) = Td(1:3,1:3);
Rc(1:3,1:3) = Tc(1:3,1:3);

Pd = Td(1:3,4);
Pc = Tc(1:3,4);

pos_error = Pd - Pc;

rot_error = rotationError(Rd,Rc);

error_vector = [pos_error;rot_error];

end