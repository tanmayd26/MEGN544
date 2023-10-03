function [theta1, theta2, d3, theta4, theta5, theta6] = inversekinematics(pose)

a = 1;
b =0.5;
theta2_1 = atan(sqrt((a+b)^2-((pose(1,4))^2+(pose(2,4))^2))/sqrt(((pose(1,4))^2)+(pose(2,4))^2-(a-b)^2));
theta2_2 = -atan(sqrt((a+b)^2-((pose(1,4))^2+(pose(2,4))^2))/sqrt(((pose(1,4))^2)+(pose(2,4))^2-(a-b)^2));
theta1_1 = atan2(pose(2,4),pose(1,4))-atan2(b*sin(theta2_1), a+b*cos(theta2_1)); 
theta1_2 = atan2(pose(2,4),pose(1,4))-atan2(b*sin(theta2_1), a+b*cos(theta2_1)); 
d3 = pose(3,4);
theta5_1 = atan2(sqrt((pose(1,3))^2+(pose(2,3))^2),pose(3,3));
theta5_2 = atan2(-sqrt((pose(1,3))^2+(pose(2,3))^2),pose(3,3));

theta4_1 = atan2(-pose(2,3)/sin(theta5_1),-pose(1,3)/sin(theta5_1))-theta1_1-theta2_1;
theta4_2 = atan2(-pose(2,3)/sin(theta5_1),-pose(1,3)/sin(theta5_1))-theta1_2-theta2_2;
theta4_3 = atan2(-pose(2,3)/sin(theta5_2),-pose(1,3)/sin(theta5_2))-theta1_1-theta2_1;
theta4_4 = atan2(-pose(2,3)/sin(theta5_2),-pose(1,3)/sin(theta5_2))-theta1_2-theta2_2;
theta6_1 = atan2(-pose(3,2)/sin(theta5_1),pose(3,1)/sin(theta5_1));
theta6_2 = atan2(-pose(3,2)/sin(theta5_2),pose(3,1)/sin(theta5_2));

% Initialize arrays to store valid angles
valid_theta4 = [];
valid_theta6 = [];

if theta5_1 || theta5_2 < 1e-15
    
    if is_valid(theta4_1, theta6_1)
        valid_theta4 = [valid_theta4, theta4_1];
        valid_theta6 = [valid_theta6, theta6_1];
    end
    if is_valid(theta4_2, theta6_2)
        valid_theta4 = [valid_theta4, theta4_2];
        valid_theta6 = [valid_theta6, theta6_2];
    end
    alpha1 = atan2(-pose(1,2),pose(2,2));
    alpha2 = -atan2(-pose(1,2),pose(2,2));

    b = (2*valid_theta4+)

end 

