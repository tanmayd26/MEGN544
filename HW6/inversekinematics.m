function [q_best] = inversekinematics(pose,p_curr)

a = 1;
b =0.5;
x = (a+b)^2;
y = (a-b)^2;
px_2 = pose(1,4)^2;
py_2 = pose(2,4)^2;
theta2_1 = real(2*atan(sqrt((x -(px_2+py_2))/((px_2+py_2)-y))));
theta2_2 = -real(2*atan(sqrt((x -(px_2+py_2))/((px_2+py_2)-y))));
theta1_1 = atan2(pose(2,4),pose(1,4))-atan2(b*sin(theta2_1), a+b*cos(theta2_1)); 
theta1_2 = atan2(pose(2,4),pose(1,4))-atan2(b*sin(theta2_2), a+b*cos(theta2_2)); 
d3 = pose(3,4);
theta5_1 = atan2(sqrt((pose(1,3))^2+(pose(2,3))^2),pose(3,3));
theta5_2 = atan2(-sqrt((pose(1,3))^2+(pose(2,3))^2),pose(3,3));

theta4_1 = atan2(-pose(2,3)/sin(theta5_1),-pose(1,3)/sin(theta5_1))-theta1_1-theta2_1;
theta4_2 = atan2(-pose(2,3)/sin(theta5_1),-pose(1,3)/sin(theta5_1))-theta1_2-theta2_2;

theta6_1 = atan2(-pose(3,2)/sin(theta5_1),pose(3,1)/sin(theta5_1));
theta6_2 = atan2(-pose(3,2)/sin(theta5_2),pose(3,1)/sin(theta5_2));



if theta5_1 < 1e-15
    
    T_03_1_1 = dhTransform(a, 0, 0, theta1_1)*dhTransform(b, 0, 0, theta2_1)*dhTransform(0,d3,0, 0);
    T_03_1_2 = dhTransform(a, 0, 0, theta1_2)*dhTransform(b, 0, 0, theta2_2)*dhTransform(0,d3,0, 0);

    T_06 = pose;

    T_36_1_1 = T_03_1_1'*T_06;
    r_33_1_1 = T_36_1_1(3,3);
    T_36_1_2 = T_03_1_2'*T_06;
    r_33_1_2 = T_36_1_2(3,3);

    alpha_1 = atan2(-T_36_1_1(1,2),T_36_1_1(2,2));
    alpha_2 = atan2(-T_36_1_2(1,2),T_36_1_2(2,2));
    A_1_1 = [2 0 1;
        0 2 1;
        1 r_33_1_1 0];
    b_1_1 =[2*p_curr(4); 2*p_curr(6); alpha_1];
    x_1_1 = (A_1_1\b_1_1);

    A_1_2 = [2 0 1;
        0 2 1;
        1 r_33_1_2 0];
    b_1_2 =[2*p_curr(4); 2*p_curr(6); alpha_2];
    x_1_2 = (A_1_2\b_1_2);
    t_1 = [theta1_1; theta2_1;d3;x_1_1(1,1); theta5_1; x_1_1(2,1)];
    t_2 = [theta1_1; theta2_2;d3;x_1_1(1,1);theta5_1;x_1_1(2,1)];
    t_3 = [theta1_1; theta2_1;d3;x_1_1(1,1);theta5_2;x_1_1(2,1)];
    t_4 = [theta1_1; theta2_2;d3;x_1_1(1,1);theta5_2;x_1_1(2,1)];
    
    % diff_1 = norm(t_1-p_curr);
    % diff_2 = norm(t_2-p_curr);
    % diff_3 = norm(t_3-p_curr);
    % diff_4 = norm(t_4-p_curr);

    % diff = [diff_1, diff_2, diff_3, diff_4];
    % [val,index] = min(diff);
    % 
    % if index == 1
    % q_best = t_1;
    % 
    % elseif index == 2
    % q_best = t_2;
    % 
    % elseif index == 3
    % q_best = t_3;
    % else
    % q_best = t_4;
    % end
    
    else

    t_1 = [theta1_1;theta2_1;d3;theta4_1; theta5_1; theta6_1];
    t_2 = [theta1_1;theta2_2;d3;theta4_1; theta5_1; theta6_1];
    t_3 = [theta1_1;theta2_1;d3;theta4_1; theta5_2; theta6_1];
    t_4 = [theta1_1;theta2_2;d3;theta4_1; theta5_2; theta6_1];
end

    diff_1 = norm(t_1-p_curr);
    diff_2 = norm(t_2-p_curr);
    diff_3 = norm(t_3-p_curr);
    diff_4 = norm(t_4-p_curr);

diff = [diff_1, diff_2, diff_3, diff_4];
[val,index] = min(diff);

if index == 1
    q_best = t_1;

elseif index == 2
    q_best = t_2;

elseif index == 3
    q_best = t_3;
else
    q_best = t_4;
end

end

