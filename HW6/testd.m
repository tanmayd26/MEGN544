T0 = [1 0 0 1;
0 0 -1 0.25;
0 1 0 0.25;
0 0 0 1];
Tf = [ 0 -1 0 0.5;
1 0 0 -0.5;
0 0 1 0;
0 0 0 1];
t_init = 0;
t_final = 2;
N = 200;
tList = linspace(t_init,t_final,N);
initial_pose_inv = inversekinematics(T0, zeros(6,1));

twist_0 = transform2Twist(T0);
twist_f = transform2Twist(Tf);
[a0, a1, a2, a3] = cubiccoeficient(twist_0,twist_f,t_final-t_init);
twistList = cubicinterpolation(a0,a1,a2,a3,t_init,tList);
pList = zeros(3,N);
thList = zeros(6,N);
for i=1:N
T_tmp_d = twist2Transform(twistList(:,i));
if i>1
joint = inversekinematics(T_tmp_d,thList(:,i-1));
else
joint = inversekinematics(T_tmp_d,initial_pose_inv(:,i));
end
thList(:,i) = joint';
T_tmp = forwardkinematics(joint(1,1),joint(2,1),joint(3,1),joint(4,1),joint(5,1),joint(5,1));
pList(:,i) = T_tmp(1:3,4);
drawArm(thList(1,i),thList(2,i),thList(3,i),thList(4,i),thList(5,i),thList(6,i),1,Tf);
pause(1/N);
end

