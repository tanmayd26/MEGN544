% cubicinterpolation: Returns the poses
%
% q = cubicinterpolation(a0,a1,a2,a3,t0,t) Gives us the poses depending on
% parameters.
%
% output1 = Pose
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = Parameter 1
% input2 = Parameter 2
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 3rd 2023
function q = cubicinterpolation(a0,a1,a2,a3,t0,t)

    q = a0 + a1*(t-t0) + a2*(t-t0).^2 + a3*(t-t0).^3;
end

