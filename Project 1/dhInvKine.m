% dhInvKine: Returns the parameter list necessary to achieve a desired homogenous transform 
% and the residual error in that transform.
% 
% [paramList, error] = dhInvKine (linkList, desTransform, paramListGuess)
% 
% output1 = residual error in the transform
% output2 = parameter list necessary to achieve desired homogenous transform
%
% input1 = linkList(the list of joint parameters created with createLink)
% input2 = desired homogenous transform(4-by-4)
% input3 = paramListGuess(initial guess at the parameters)

% Tanmay Desai
% 10922557
% MEGN 544A
% November 5th 2023

function [paramList, error] = dhInvKine( linkList,desTransform, paramListGuess)
%% initializing
paramList = paramListGuess;
Tc = dhFwdKine(linkList,paramList);
error = transError(desTransform,Tc);
dp = Inf;
%%

while norm(error) > 1e-15 && norm(dp)>1e-15
    Jv = velocityJacobian(linkList,paramList);
    [U,S,V]=svd(Jv);
    [r,c]=size(S);
    maxS = max(S);
    Inv_s = zeros(size(S));

    for i = 1:r
        if S(i,i)< maxS(1,1)/500
            Inv_s(i,i) = 0;
        else
            Inv_s(i,i) = 1/S(i,i);
        end
    end
    InvJv= V*Inv_s'*U';
    dp=InvJv*error;
    paramList = paramList+dp;
    Tc=dhFwdKine(linkList,paramList);
    error=transError(desTransform,Tc);
end
end