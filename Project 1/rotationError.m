% rotationError: Returns 3x1 vector an angle*axis vector
% 
% [rot_error_vector] = rotationError(Rot_desired, Rot_current)
% Returns 3x1 vector an angle*axis vector
% 
% 
% output1 =rot_error_vector is a 3x1 vector describing the axis of rotation multiplied by the angle of rotation
%
% input1 = is the rotation matrix describing the desired coordinate frame (of the robot for example) in the reference frame 
%
% input2 =is the rotation matrix describing the current coordinate frame (of the robot for example) in the reference frame (of the world  for example).
%
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023
function rot_error_vector = rotationError(Rot_desired,Rot_current)

err = Rot_desired*Rot_current';

angleaxis = rot2AngleAxis(err);


rot_error_vector = angleaxis;

end