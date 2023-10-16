% dhFwdKine: Returns the forward kinematics of a manipulator
% with the provided DH parameter set.
%
% H = dhFwdKine(linkList, paramList) Provides us the 3-by-3 roation matrix for the  angle 
% axis represenation for a rotation of theta about k vector 
%
% output1 = 3-by-3 rotation matrix
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = Axis parameter i.e, k being 3-by-1 vector
% input2 = Angle parameter theta in radians 
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 22nd 2023
function H = dhFwdKine(linkList, paramList)

N = length(linkList);
H = eye(4);

for i=1:N
    if linkList(i).isRotary == 1
        H = H*dhTransform(linkList(i).a,linkList(i).d,linkList(i).alpha,paramList(i)-linkList(i).offset);
    else
        H = H*dhTransform(linkList(i).a,paramList(i)-linkList(i).offset,linkList(i).alpha,linkList(i).theta);

    end

end

end
 
    