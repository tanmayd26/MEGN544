% rot2RPY: Returns the roll ,pitch & yaw for the end effector for the given
% rotation matrix
%
% [roll, pitch, yaw] = rot2RPY(R) Provides us the roll,pitch and yaw angle
% for a 3-by-3 input roation matrix with a special case of pitch=pi/2
%
% output1 = 2-by-1 Roll in radian for the end effector 
% output2 = 2-by-1 Pitch in radian for the end effector
% output3 = 2-by-1 Yaw in radian for the end effector
%
% input1 = 3-by-3 rotation matrix 
% input2 = description of what the second input is/means include units if appropriate 
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 1st 2023
function [roll,pitch,yaw] = rot2RPY(R)

pitch_p = atan2(-R(3,1),sqrt((R(1,1))^2+(R(2,1))^2));
pitch_n = atan2(-R(3,1),-sqrt((R(1,1))^2+(R(2,1))^2));
if pitch_p == pi/2 || pitch_p == -pi/2  % Check for near-pi/2 pitch (gimbal lock)
        if(pitch_p == pi/2)
        roll_p = atan2(R(1,2), R(2,2));
        roll_n = atan2(R(1,2), R(2,2));
        else
        roll_p = atan2(-R(1,2), R(2,2));
        roll_n = atan2(-R(1,2), R(2,2));
        end
        yaw_p = 0;
        yaw_n = 0;

else
        roll_p = atan2(R(3,2)/cos(pitch_p),R(3,3)/cos(pitch_p));
        roll_n = atan2(R(3,2)/cos(pitch_n),R(3,3)/cos(pitch_n));
        yaw_p = atan2(R(2,1)/cos(pitch_p),R(1,1)/cos(pitch_p));
        yaw_n = atan2(R(2,1)/cos(pitch_n),R(1,1)/cos(pitch_n));
end
pitch = [pitch_p;
    pitch_n];
roll = [roll_p;
    roll_n];
yaw = [yaw_p;
    yaw_n];


end