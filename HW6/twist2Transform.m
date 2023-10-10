% twist2Transform: Returns the homogenous transformation matrix corresponding 
% to a 6 element twist vector. 
%
% [ H ] = twist2Transform( t ) for a given 6-by-1 twist vector stacked
% as[v;w th] the resulting functions returns a homogenous tranformation
% matrix of 4-by-4 with rotation and displacement calculated seperately
%
% output1 = 4-by-4 homogenous tranformation matrix
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = 6-by-1 twist vector stacked as[v;w th] 
% input2 = description of what the second input is/means include units if appropriate 
%
% Tanmay Desai
% 10922557
% MEGN 544 
% October 1st 2023
function[H] = twist2Transform(t)

%separting the v and w vector
v=t([1 2 3]);
O=t([4 5 6]); % Omega
% Special case if there is no rotation i.e. O vector is zero then theta=0 
% else theta=norm(O)
if O == 0 
    theta=0;
else
	theta  = norm(O); % calculate theta
    O=O./theta; % gives k hat 
end
if theta==0
    R=eye(3);  %roation matrix = 3-by-3 identity matrix
    d=v; % d is equal to v for no ratation
else
%if theta jot equal to 0,calculate skew symmetric of Omega vector
    O_x=[0 -O(3) O(2); O(3) 0 -O(1); -O(2) O(1) 0];
    l=(1-cos(theta)); 
    R=cos(theta).*eye(3)+(l.*(O*O'))+sin(theta).*O_x; %3-by-3 rotation matrix using O_x,O and theta
    d=((eye(3)-R)*O_x+O*O'.*theta)*v; % 3-by-1 displacement matrix 
end
H=[R,d];
H=[H;[zeros(1,3),1]];  %getting a 4-by-4 homogenous transformation matrixfor the twist vector

end