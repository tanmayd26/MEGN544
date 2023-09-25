% twist2Transform: Returns the twist vector corresponding to the provided homogenous
% transform matrix. 
%
% [ t ]= transform2Twist( H ) for a given 4-by-4 homogenous tranformation matrix 
%  the resulting functions returns a 6-by-1 twist vector stacked
% as[v;w th] and v and w calculated seperately
%
% output1 = 6-by-1 twist vector stacked as[v;w th] 
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = 4-by-4 homogenous tranformation matrix
% input2 = description of what the second input is/means include units if appropriate 
%
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023
function [t] = transform2Twist(H)

% seperating R and d matrix from H
 R = H(1:3,1:3);
 d = H(1:3,4);
theta=acos((R(1,1)+R(2,2)+R(3,3)-1)/2); % theta calculation
%special cased if theta=o then v=d and w=0 vector 
if theta==0
v=d;
k=[zeros(3,1)];
t=[v;k];
else
    %if theta not equal to 0 then w vector is calulated as below using
    %theta value
k1=(R(3,2)-R(2,3))/(2*sin(theta));
k2=(R(1,3)-R(3,1))/(2*sin(theta));
k3=(R(2,1)-R(1,2))/(2*sin(theta));

k=[k1,k2,k3];
k=k(:);
k_x=[0,-k(3),k(2);
    k(3),0,-k(1);
    -k(2),k(1),0];
v=inv(((eye(3)-R)*k_x+theta*k*k'))*d ;% v vector calculated using R and d
omega=k.*theta; % twist vector is v and omega ,where omega=w*theta
t=[v;omega]; %6-by-1 twist vector
end
end