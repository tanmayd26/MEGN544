theta = [0,pi/3, pi/4, pi/4, pi/3, pi/3];

    alpha = [pi/2, 0,-pi/2,pi/2,-pi/2,0];
    a = [0, 0.270, 0.070, 0, 0 ,0];
    d = [0.290, 0 ,0 , 0.302, 0 , 0.072];
T = cell(1, 6);
    for i=1:6
        H = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
            sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
            0, sin(alpha(i)), cos(alpha(i)), d(i);
            0, 0, 0, 1];
        T{i} = H;
    end 

    T_06_fk = eye(4);
    for i=1:6
        T_06_fk = T_06_fk*T{i};
    end

T_curr = T_06_fk;

 

 % Theta 1
d_56 =(d(6)*T_curr(1:3,1:3)*[0;0;1]);
d_06 = T_curr(1:3,4);
d_05 = d_06 - d_56;

theta1 = pi+ atan2(d_05(2,1),d_05(1,1));


% Theta 3
rot_01 = rotZ(theta1)*rotX(alpha(1));

d_01 = d(1)*[0;0;1];
rot_10 = inv(rot_01);

d_14 = (rot_10)*(d_05-d_01);

phi = atan2(d(4),a(3));
d_24 = sqrt((d(4)^2)+(a(3)^2));


si(1) = 2 * atan2(sqrt((2*a(2)*d_24)+(d_14(1,1)^2)+(d_14(2,1)^2)-(a(2)^2)-(d_24^2)),abs(sqrt((2*a(2)*d_24)-(d_14(1,1)^2)-(d_14(2,1)^2)+(a(2)^2)+(d_24^2))));

theta3 = pi - si - phi;
% Theta 2
alpha1 = atan2(d_14(2,1),d_14(1,1));

gamma = atan2((d_24*sin(theta3+phi)),(a(2)+(d_24*cos(theta3+phi))));

theta2 = alpha1 - gamma;
% Theta 5
rot_03 = rot_01*rotZ(theta2)*rotX(0)*rotZ(theta3)*rotX(-pi/2);

rot_36 = (inv(rot_03))*T_curr(1:3,1:3);

R1 = rot_36;

theta5 = atan2(sqrt((R1(1,3)^2)+(R1(2,3)^2)),R1(3,3));

% Theta 4%6
if sin(theta5) == 0 

    
        theta4=0;
        theta6=atan2(R1(2,1),cos(theta5)*R1(1,1));
 
else
theta4 = atan2((-R1(2,3)/sin(theta5)),(-R1(1,3)/sin(theta5)));

theta6 = atan2((-R1(3,2)/sin(theta5)),(R1(3,1)/sin(theta5)));

end