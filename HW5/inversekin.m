
%find the inverse solution
function [theta1, theta2, theta3, theta4, theta5, theta6] = inversekin(a2, a3, d4, T06)
        alpha = [pi/2, 0,-pi/2,pi/2,-pi/2,0];
        a = [0, 0.270, 0.070, 0, 0 ,0];
        d = [0.290, 0 ,0 , 0.302, 0 , 0.072];
        yf = T06(2,4);
        xf = T06(1,4);

        d_05 = T06(1:3,4) - (d(6)*T06(1:3,1:3)*[0;0;1]);
        R06 = [T06(1,1:3); T06(2,1:3); T06(3,1:3)];
        theta1 = atan2(yf, xf);
        l = sqrt(a3^2 + d4^2);
        R01 = rotZ(theta1)*rotX(pi/2);
        d_01 = d(1)*[0;0;1];
        d_14 = inv(R01)*(d_05-d_01);
        gamma = atan2(d4, a3);
        psi = 2 * atan2(sqrt((2*a(2)*l)+(d_14(1,1)^2)+(d_14(2,1)^2)-(a(2)^2)-(l^2)),sqrt((2*a(2)*l)-(d_14(1,1)^2)-(d_14(2,1)^2)+(a(2)^2)+(l^2)));
        theta3 = pi - gamma - psi;
        theta2 = atan2(d_14(2,1),d_14(1,1)) - atan2(l*sin(psi+theta3), a2 + l*(cos(psi+theta3)));
        R01 = rotZ(theta1)*rotX(pi/2);
        R12 = rotZ(theta2);
        R23 = rotZ(theta3)*rotX(-pi/2);
        R03 = R01*R12*R23;
        R30 = inv(R03);
        R36 = R30 * R06;
        theta5 = atan2(sqrt(R36(1,3)^2+R36(2,3)), R36(3,3));
        theta4 = atan2(-R36(2,3)/sin(theta5), -R36(1,3)/sin(theta5));
        theta6 = atan2(R36(3,2)/sin(theta5), -R36(3,1)/sin(theta5));
       

end