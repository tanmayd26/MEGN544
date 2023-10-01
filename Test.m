R = [-0.987466562219235	-0.154630595036917	0.0316096121054145;
-0.154630595036917	0.907746409222729	-0.389981840117934;
0.0316096121054143	-0.389981840117934	-0.920279847003495];
r1 = R(3,2)-R(2,3);
r2 = R(1,3)-R(3,1);
r3 =  R(2,1)-R(1,2);

Y = (1/2)*norm([r1;r2;r3]);

X = (trace(R)-1)/2;

theta = atan2(Y,X);

sintheta = sin(theta);
   if (theta~=pi && theta~=-pi && theta~=0)
        k = 1/(2*sintheta)*[r1;r2;r3];
   elseif (theta==0)
       k=[0;0;0];
   elseif (R(1,1) ~=1 || R(2,2)~=1 ||R(3,3)~=1)
       k_kT = (R+eye(3))/2;
       k1 = sqrt(k_kT(1,1));
       k2 = sqrt(k_kT(2,2));
       k3 = sqrt(k_kT(3,3));
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
   else 
      k = [k1;-k2;k3];
    end
   

    % Construct the angle-axis representation
    Omega = theta * k;