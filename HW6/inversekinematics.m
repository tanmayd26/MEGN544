function [theta1, theta2, d3, theta4, theta5, theta6] = inversekinematics(pose)

a = 1;
b =0.5;
theta2 = atan(sqrt((a+b)^2-((pose(1,4))^2+(pose(2,4))^2))/sqrt(((pose(1,4))^2)+(pose(2,4))^2-(a-b)^2));
theta1 = atan2(pose(2,4),pose(1,4))-atan2(b*sin(theta2), a+b*cos(theta2)); 
d3 = pose(3,4);
theta5 = atan2(sqrt((pose(1,3))^2+(pose(2,3))^2),pose(3,3));
theta4 = atan2(-pose(2,3)/sin(theta5),-pose(1,3)/sin(theta5))-theta1-theta2;
theta6 = atan2(-pose(3,2)/sin(theta5),pose(3,1)/sin(theta5));