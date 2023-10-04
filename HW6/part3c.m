initial_pose=[1 0 0 1;0 0 -1 0.25;0 1 0 0.25;0 0 0 1];
final_pose=[0 -1 0 0.5;1 0 0 -0.5;0 0 1 0;0 0 0 1];
%% Joint Spapce Interpolation
t0=0;
T=2;
t=linspace(t0,T,200);
R_init = initial_pose(1:3,1:3);
R_final = final_pose(1:3,1:3);
%% Quaternion
q0_init=sqrt((trace(R_init)+1)/4);
q1_init=(R_init(3,2)-R_init(2,3))/(4*q0_init);
q2_init=(R_init(1,3)-R_init(3,1))/(4*q0_init);
q3_init=(R_init(2,1)-R_init(1,2))/(4*q0_init);
q_init=[q1_init,q2_init,q3_init];
Q_init=[q0_init;q1_init;q2_init;q3_init];
% Final Quternion
q0_final=sqrt((trace(R_final)+1)/4);
q1_final=(R_final(3,2)-R_final(2,3))/(4*q0_final);
q2_final=(R_final(1,3)-R_final(3,1))/(4*q0_final);
q3_final=(R_final(2,1)-R_final(1,2))/(4*q0_final);
q_f=[q1_final,q2_final,q3_final];
Q_f=[q0_final;q1_final;q2_final;q3_final];
% Interpolate Quaternions
[a0_om_x,a1_om_x,a2_om_x,a3_om_x] =cubiccoeficient(tf,Q_0(1),Q_f(1));
[a0_om_y,a1_om_y,a2_om_y,a3_om_y] =GenCoffs(0,tf,Q_0(2),Q_f(2));
[a0_om_z,a1_om_z,a2_om_z,a3_om_z] =GenCoffs(0,tf,Q_0(3),Q_f(3));
[a0_w,a1_w,a2_w,a3_w] =GenCoffs(0,tf,Q_0(4),Q_f(4));
for i=1:length(t)
    Q_t_x=a0_om_x+a1_om_x*t(i)+a2_om_x*t(i).^2+a3_om_x*t(i).^3;
    Q_t_y=a0_om_y+a1_om_y*t(i)+a2_om_y*t(i).^2+a3_om_y*t(i).^3;
    Q_t_z=a0_om_z+a1_om_z*t(i)+a2_om_z*t(i).^2+a3_om_z*t(i).^3;
    Q_t_w=a0_w+a1_w*t(i)+a2_w*t(i).^2+a3_w*t(i).^3;
    Q_t=[Q_t_x;Q_t_y;Q_t_z;Q_t_w];
    %
    % Reconstract The Homogeneous Matrix
    q0=Q_t(1);
    q=[Q_t(2);Q_t(3);Q_t(4)];
    q_x=[0 -q(3) q(2);
    q(3) 0 -q(1);
    -q(2) q(1) 0];
    R=(q0^2-(q'*q))*eye(3)+2*q0*q_x+2*(q*q');
    d_t=[dx(i);dy(i);dz(i)];
    T_t=[R_t,d_t;0 0 0 1];
    joints=inverseKinematics(T_t);
    Joint_Variable(i,:,3)=joints;
end