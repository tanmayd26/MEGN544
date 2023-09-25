% rot2AngleAxis: Returns the angle and axis corresponding to a rotation matrix.
%
% [Omega] = rot2AngleAxis(R) Provides us with an angle axis
% represenation for the 3-by-3 input roation matrix with a special case of
% theta=pi
%
% output1 = Angle-Axis i.e, Omega being 3-by-1 vector
% output2 = description of what the second input is/means include units if appropriate 
%
% input1 = 3-by-3 rotation matrix 
% input2 = description of what the second input is/means include units if appropriate 
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 1st 2023

function[Omega]= rot2AngleAxis(R)
    R_sub = [R(3,2)-R(2,3);
            R(1,3)-R(3,1);
            R(2,1)-R(1,2)];
    Y = (1/2)*norm(R_sub);

    X = (trace(R)-1)/2;

    theta = atan2(Y,X);
   if theta == pi % Check if theta is very close to pi
        % Handle the case when the rotation angle is pi
        k = (1/2)*[R(1,1)+1; R(2,2)+1;R(3,3)+1]; % Depending on the axis of rotation.
        
   elseif (theta==0)
       k=[0;0;0];
   else
        k = (1/(2*sin(theta)))*R_sub;
    end
   

    % Construct the angle-axis representation
    Omega = theta * k;
end
