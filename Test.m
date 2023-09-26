R = [-0.987466562219235	-0.154630595036917	0.0316096121054145;
-0.154630595036917	0.907746409222729	-0.389981840117934;
0.0316096121054143	-0.389981840117934	-0.920279847003495];
r1 = R(3,2)-R(2,3);
r2 = R(1,3)-R(3,1);
r3 =  R(2,1)-R(1,2);
 R_sub = [r1;
            r2;
           r3];
    Y = (1/2)*norm(R_sub);

    X = (trace(R)-1)/2;

    theta = atan2(Y,X);
   if (theta~=pi && theta~=-pi && theta~=0)
        k = (1/(2*sin(theta)))*R_sub;
   elseif (theta==0)
       k=[0;0;0];
   else
       scale = sqrt(1 + trace(R));
        k1 = (R(3,2) - R(2,3)) / (2 * scale);
        k2 = (R(1,3) - R(3,1)) / (2 * scale);
        k3 = (R(2,1) - R(1,2)) / (2 * scale);
        k = [k1; k2; k3];
    end
   

    % Construct the angle-axis representation
    Omega = theta * k;