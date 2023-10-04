%% Quaternion
q0_i=sqrt((trace(R0)+1)/4);
q1_i=(R0(3,2)-R0(2,3))/(4*q0_i);
q2_i=(R0(1,3)-R0(3,1))/(4*q0_i);
q3_i=(R0(2,1)-R0(1,2))/(4*q0_i);
q_i=[q1_i,q2_i,q3_i];
Q_0=[q0_i;q1_i;q2_i;q3_i];
% Final Quternion
q0_f=sqrt((trace(Rf)+1)/4);
q1_f=(Rf(3,2)-Rf(2,3))/(4*q0_f);
q2_f=(Rf(1,3)-Rf(3,1))/(4*q0_f);
q3_f=(Rf(2,1)-Rf(1,2))/(4*q0_f);
q_f=[q1_f,q2_f,q3_f];
Q_f=[q0_f;q1_f;q2_f;q3_f];
% Interpolate Quaternions
[a0_om_x,a1_om_x,a2_om_x,a3_om_x] =GenCoffs(0,tf,Q_0(1),Q_f(1));
[a0_om_y,a1_om_y,a2_om_y,a3_om_y] =GenCoffs(0,tf,Q_0(2),Q_f(2));
[a0_om_z,a1_om_z,a2_om_z,a3_om_z] =GenCoffs(0,tf,Q_0(3),Q_f(3));
[a0_w,a1_w,a2_w,a3_w] =GenCoffs(0,tf,Q_0(4),Q_f(4));
for it=1:length(t)
    Q_t_x=a0_om_x+a1_om_x*t(it)+a2_om_x*t(it).^2+a3_om_x*t(it).^3;
    Q_t_y=a0_om_y+a1_om_y*t(it)+a2_om_y*t(it).^2+a3_om_y*t(it).^3;
    Q_t_z=a0_om_z+a1_om_z*t(it)+a2_om_z*t(it).^2+a3_om_z*t(it).^3;
    Q_t_w=a0_w+a1_w*t(it)+a2_w*t(it).^2+a3_w*t(it).^3;
    Q_t=[Q_t_x;Q_t_y;Q_t_z;Q_t_w];
    %
    % Reconstract The Homogeneous Matrix
    q0=Q_t(1);
    q=[Q_t(2);Q_t(3);Q_t(4)];
    q_x=[0 -q(3) q(2);
    q(3) 0 -q(1);
    -q(2) q(1) 0];
    R=(q0^2-(q'*q))*eye(3)+2*q0*q_x+2*(q*q');
    d_t=[dx(it);dy(it);dz(it)];
    T_t=[R_t,d_t;0 0 0 1];
    joints=IKfnc(T_t);
    Joint_Variable(it,:,3)=joints;
end