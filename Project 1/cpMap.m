% cpMap: Returns the matrix packing of the cross product operator 
%
% [ X ] = cpMap(w) For a given vector w the function returns a matrix representing a packing 
% of the cross product operator
%
% output1 = 3-by-3 matrix  
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = w is a 3-by-1 vector 
% input2 = description of what the second input is/means include units if appropriate 
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 1st 2023

%Should I check if the input matrix is 3 elements only?
function [X] = cpMap(w)


X = [0 -w(3) w(2);
    w(3) 0 -w(1);
    -w(2) w(1) 0];
end