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
r1 = R(3,2)-R(2,3);
r2 = R(1,3)-R(3,1);
r3 =  R(2,1)-R(1,2);

Y = (1/2)*norm([r1;r2;r3]);

X = (trace(R)-1)/2;

theta = atan2(Y,X);

sintheta = sin(theta);
   k_kT = (R+eye(3))/2;
       k1 = sqrt(k_kT(1,1));
       k2 = sqrt(k_kT(2,2));
       k3 = sqrt(k_kT(3,3));
   if (theta~=pi && theta~=-pi && theta~=0)
        k = 1/(2*sintheta)*[r1;r2;r3];
   elseif (theta==0)
       k=[0;0;0];
   elseif (theta == pi)
      k = [k1;-k2;k3];
   else
    
    if abs(R(1, 2)) > eps
        k2_sign = R(1, 2) / abs(R(1, 2));
        k2 = k2_sign * k2;
    end

    if abs(R(1, 3)) > eps
        k3_sign = R(1, 3) / abs(R(1, 3));
        k3 = k3 * k3_sign;
    end

    if abs(R(2, 1)) > eps
        k1_sign = R(2, 1) / abs(R(2, 1));
        k1 = k1_sign * k1;
    end
       k = [k1;k2;k3];
   
    end
   

    % Construct the angle-axis representation
    Omega = theta * k;
end
