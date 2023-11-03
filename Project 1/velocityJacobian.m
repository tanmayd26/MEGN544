% velocityJacobian: Returns the velocity jacobian of the manipulator given an array of links created by the createLink function
% and the current joint variables
% Jv – the velocity jacobian
% Jv – the velocity jacobian
% JvDot – the time derivative of the velocity jacobian
% linkList – the list of joint parameters created with
% createLink
% paramList – the current theta or d values for the
% joints. (an Nx1 array)
% paramRateList – the current theta_dot and d_dot
% values for the joints. (an Nx1 array)
% If paramRateList is not provided (check with
% exist(‘paramRateList’,’var’) ), then return [] for
% JvDot. Otherwise calculate JvDot as well.

% [Jv, JvDot] = velocityJacobian( linkList, paramList, paramRateList) Returns the forward kinematics of a manipulator
% with the provided DH parameter set

% output1 = velocity jacobian
% output2 = time derivative of the velocity jacobian
%
% input1 = linkList(the list of joint parameters created with createLink)
% input2 = paramList(the current theta or d values for the joints(Nx1 array))
% input3= paramRateList(the current theta_dot and d_dot values for the joints(Nx1 array))

% Tanmay Desai
% 10922557
% MEGN 544A
% November 5th 2023
function [Jv, JvDot] = velocityJacobian( linkList, paramList,paramRateList)

N = length(linkList);
H = eye(4);
theta_dot=zeros(1,N);
for i=1:N
    a(i) = linkList(i).a;
    alpha(i) = linkList(i).alpha;
    offset(i) = linkList(i).offset;
    isRotary(i) = linkList(i).isRotary;
    if isRotary(i) == 1
        theta(i) = paramList(i)-offset(i);
        d(i) = linkList(i).d;
        if exist('paramRateList','var')==1
            theta_dot(i)=paramRateList(i);
        end

    else
        d(i) = paramList(i)-offset(i);
        theta(i) = linkList(i).theta;
        if exist('paramRateList','var')==1
            d_dot(i)=paramRateList(i);
        end
    end

end

for i=1:N
    T(:,:,1) = eye(4);
    T(:,:,i+1) = H*dhTransform(a(i),d(i),alpha(i),theta(i));
    H = T(:,:,i+1);
end
d = T(1:3,4,end);
for i=1:N+1
    z(:,i)=T(1:3,3,i);
    diff_d(:,i)=d-T(1:3,4,i);
end
Jv = zeros(6,N);
JvDot = zeros(6,N);
for i=1:N
    if isRotary(i)==1
        Jv(1:6,i) = [cross(z(:,i),diff_d(:,i));z(:,i)];
    else
        Jv(1:6,i)=[z(:,i);0;0;0];
    end
end
w = zeros(3,N+1);
v = zeros(3,N+1);

JvDot = zeros(6,N);
for i=1:N
    if isRotary(i) == 1
        w(:, i+1)= w(:,i) + theta_dot(i)*z(:, i);
        if exist('paramRateList','var') ==1
            v(:,i+1) = v(:,i)+cross(w(:,i+1),T(1:3,4,i+1)-T(1:3,4,i));
        end
    elseif isRotary(i)==0
        w(:,i+1) = w(:,i);
        if exist('paramRateList','var') ==1
            v(:,i+1) = v(:,i)+cross(w(:,i+1),T(1:3,4,i+1)-T(1:3,4,i))+T(1:3,3,i)*d_dot(i);
        end
    else
        w(:,i+1)=w(:,i);
        v(:,i+1)=v(:,i);
    end
end


JvDot=zeros(6,N);
for i=1:N

    if exist('paramRateList','var')
        if isRotary(i)==1
            diffv = v(:,end)-v(:,i);
            JvDot(:,i) = [cross(cross(w(:,i+1),z(:,i)),diff_d(:,i))+cross(z(:,i),diffv);cross(w(:,i+1),z(:,i))];
        else
            JvDot(:,i) = [cross(w(:,i+1),z(:,i));0;0;0];
        end
    else
        JvDot =[];
    end
end

end

