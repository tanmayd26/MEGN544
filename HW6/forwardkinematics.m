function [eepos] = forwardkinematics(theta1, theta2, d3, theta4, theta5, theta6)

eepos= zeros(4,1);
a = 1;
b = 0.5;

c_124 = cos(theta1+theta2+theta4);
s_124 = sin(theta1+theta2+theta4);

eepos = [c_124.*cos(theta5).*cos(theta6)-s_124.*sin(theta6) -s_124.*cos(theta6)-c_124.*cos(theta5).*sin(theta6) -c_124.*sin(theta5) cos(theta1+theta2).*b+cos(theta1).*a;
    s_124.*cos(theta5).*cos(theta6)+c_124.*sin(theta6) c_124.*cos(theta6)-s_124.*cos(theta5).*sin(theta6) -s_124.*sin(theta5) sin(theta1+theta2).*b+sin(theta1).*a;
    cos(theta6).*sin(theta5) -sin(theta5).*sin(theta6) cos(theta5) d3;
    0 0 0 1];