% paramRateList = -2.888831558203870;
% paramList = -2.516687052860036;
% linkList = load('linkList.mat');

% N = length(linkList);
% H = eye(4);
% theta_dot=zeros(1,N);
% for i=1:N
%     a(i) = linkList(i).a;
%     alpha(i) = linkList(i).alpha;
%     offset(i) = linkList(i).offset;
%     isRotary(i) = linkList(i).isRotary;
%     if isRotary(i) == 1
%         theta(i) = paramList(i)-offset(i);
%         d(i) = linkList(i).d;
%         if exist('paramRateList','var')
%             theta_dot(i)=paramRateList(i);
%         end
% 
%     else
%         d(i) = paramList(i)-offset(i);
%         theta(i) = linkList(i).theta;
%         if exist('paramRateList','var')
%             d_dot(i)=paramRateList(i);
%         end
%     end
% 
% end
% 
% for i=1:N
%     T(:,:,1) = eye(4);
%     T(:,:,i+1) = H*dhTransform(a(i),d(i),alpha(i),theta(i));
%     H = T(:,:,i+1);
% end
% d = T(1:3,4,end);
% for i=1:N+1
%     z(:,i)=T(1:3,3,i);
%     diff_d(:,i)=d-T(1:3,4,i);
% end
% Jv = zeros(6,N);
% JvDot = zeros(6,N);
% for i=1:N
%     if isRotary(i)==1
%         Jv(1:6,i) = [cross(z(:,i),diff_d(:,i));z(:,i)];
%     else
%         Jv(1:6,i)=[z(:,i);0;0;0];
%     end
% end
% w = zeros(3,N+1);
% v = zeros(3,N+1);
% 
% JvDot = zeros(6,N);
% for i=1:N
% 
%     if isRotary(i) == 1
%         w(:, i+1)= w(:,i) + theta_dot(i)*z(:, i); 
%         v(:,i+1) = v(:,i)+cross(w(:,i+1),T(1:3,4,i+1)-T(1:3,4,i));
%     elseif isRotary(i)==0
%         w(:,i+1) = w(:,i);
%         v(:,i+1) = v(:,i)+cross(w(:,i+1),T(1:3,4,i+1)-T(1:3,4,i))+T(1:3,3,i)*d_dot(i);
%     else
%         w(:,i+1)=w(:,i);
%         v(:,i+1)=v(:,i);
%     end
% end 
% 
% 
% JvDot=zeros(6,N);
% for i=1:N
% 
%     if exist('paramRateList','var')
%         if isRotary(i)==1
%             diffv = v(:,end)-v(:,i);
%             JvDot(:,i) = [cross(cross(w(:,i+1),z(:,i)),diff_d(:,i))+cross(z(:,i),diffv);cross(w(:,i+1),z(:,i))];
%         else
%             JvDot(:,i) = [cross(w(:,i+1),z(:,i));0;0;0];
%         end
%     else
%         JvDot =[];
%     end
% end
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
    if r<c
        N = r;
    else
        N = c;
    end
    for i = 1:N
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
