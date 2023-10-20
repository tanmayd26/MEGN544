T_des = [0.429827827654558	0.893527447285214	-0.129833506929966	-0.00348228519229711;
    -0.177170506823121	-0.0575340167941418	-0.982497047539365	-0.0938703185970968;
    -0.885357922021259	0.445307239846268	0.133576989241024	0.441540655654374;
    0	0	0	1];

th_last = [-1.34462246926484;
    -1.45975435983443;
    1.31631027597796;
    -1.62440540245517;
    0.339500573552582;
    -1.13820577400355];
a = [0;0.27;0.07;0;0;0];
d = [0.29;0;0;0.302;0;0.072];
alpha = [pi/2;0;-pi/2;pi/2;-pi/2;0];

    % Theta 1
    d_05 = T_des(1:3,4) - (d(6).*T_des(1:3,3).*[0;0;1]);
    theta1(1) = real(atan2(d_05(2,1),d_05(1,1)));
    theta1(2) = pi+real(atan2(d_05(2,1),d_05(1,1)));
    % Theta 3
    rot_01 = rotZ(theta1(1))*rotX(alpha(1));
    rot_01_2 = rotZ(theta1(2))*rotX(alpha(1));
    d_01 = d(1)*[0;0;1];
    rot_10 = rot_01';
    rot_10_2 = rot_01_2';
    d_14 = (rot_10)*(d_05-d_01);

    d_14_2 = (rot_10_2)*(d_05-d_01);
    phi = atan2(d(4),a(3));
    l = sqrt((d(4)^2)+(a(3)^2));
 
    si(1) = 2 * atan2(sqrt((2*a(2)*l)+(d_14(1,1)^2)+(d_14(2,1)^2)-(a(2)^2)-(l^2)),sqrt((2*a(2)*l)-(d_14(1,1)^2)-(d_14(2,1)^2)+(a(2)^2)+(l^2)));
    si(2) = 2 * atan2(sqrt((2*a(2)*l)+(d_14_2(1,1)^2)+(d_14_2(2,1)^2)-(a(2)^2)-(l^2)),sqrt((2*a(2)*l)-(d_14_2(1,1)^2)-(d_14_2(2,1)^2)+(a(2)^2)+(l^2)));
    theta3(1:2) = pi - si(1:2) - phi;
    % Theta 2
    alpha1(1) = atan2(d_14(2,1),d_14(1,1));
    alpha1(2) = atan2(d_14_2(2,1),d_14_2(1,1));
    gamma(1) = atan2((l*sin(theta3(1)+phi)),(a(2)+(l*cos(theta3(1)+phi))));
    gamma(2) = atan2((l*sin(theta3(2)+phi)),(a(2)+(l*cos(theta3(2)+phi))));
    theta2(1:2) = alpha1(1:2) - gamma(1:2) + pi/2;
    % Theta 5
    rot_03 = rot_01*rotZ(theta2(1))*rotX(0)*rotZ(theta3(1))*rotX(-pi/2);
    rot_03_2 = rot_01_2*rotZ(theta2(2))*rotX(0)*rotZ(theta3(2))*rotX(-pi/2);
    rot_36 = (rot_03')*T_des(1:3,1:3);
    rot_36_2 = (rot_03_2')*T_des(1:3,1:3);
    R1 = rot_36;
    R2 = rot_36_2;
    theta5(1) = atan2(sqrt((R1(3,2)^2)+(R1(3,1)^2)),R1(3,3));
    theta5(2) = atan2(sqrt((R2(3,2)^2)+(R2(3,1)^2)),R2(3,3));
    % Theta 4%6
    if sin(theta5(1)) == 0 || sin(theta5(2)) == 0


        theta4(1:2)=0;
        theta6(1)=atan2(R1(2,1),cos(theta5(1))*R1(1,1));
        theta6(2)=atan2(R1(2,1),cos(theta5(2))*R1(1,1));
    else
        theta4(1) = atan2((-R1(2,3)/sin(theta5(1))),(-R1(1,3)/sin(theta5(1))));
        theta4(2) = atan2((-R2(2,3)/sin(theta5(2))),(-R2(1,3)/sin(theta5(2))));
        theta6(1) = atan2((-R1(3,2)/sin(theta5(1))),(R1(3,1)/sin(theta5(1))));
        theta6(2) = atan2((-R2(3,2)/sin(theta5(2))),(R2(3,1)/sin(theta5(2))));
    end
    %
    th1(1:4,1) = theta1(1);
    th1(5:8,1) = theta1(2);

    th3(1:2,1) = theta3(1);
    th3(3:4,1) = theta3(2);
    th3(5:6,1) = theta3(1);
    th3(7:8,1) = theta3(2);

    th2(1:2,1) = theta2(1);
    th2(3:4,1) = theta2(2);
    th2(5:6,1) = theta2(1);
    th2(7:8,1) = theta2(2);

    for i=1:8
        if(i/2==0)
            th5(i,1) = theta5(2);
            th4(i,1) = theta4(2);
            th6(i,1) = theta6(2);
        else
            th5(i,1) = theta5(1);
            th4(i,1) = theta4(1);
            th6(i,1) = theta6(1);
        end
    end
    if isreal(th1)==1 && isreal(th2)==1 && isreal(th3)==1 && isreal(th4)==1 && isreal(th5)==1 && isreal(th6)==1
        reachable=1;
    else
        reachable=0;
    end
    %
    %
    %
