%% Twist
R0=T0d(1:3,1:3);
Rf=Tfd(1:3,1:3);
om0=rot2AngleAxis(R0);
%
omf=rot2AngleAxis(Rf);
% Interpolatting Omega
[a0_om_x,a1_om_x,a2_om_x,a3_om_x] =GenCoffs(0,tf,om0(1),omf(1));
[a0_om_y,a1_om_y,a2_om_y,a3_om_y] =GenCoffs(0,tf,om0(2),omf(2));
[a0_om_z,a1_om_z,a2_om_z,a3_om_z] =GenCoffs(0,tf,om0(3),omf(3));
% Interpolating position
d0=T0d(1:3,4);
df=Tfd(1:3,4);
% Initial v
th0_Twist=atan2(0.5*norm([R0(3,2)-R0(2,3);R0(1,3)-R0(3,1);R0(2,1)-R0(1,2)]),0.5*(trace(R0)-1));
K0_Twist= [0;0;0];
K0_x=[0 -K0_Twist(3) K0_Twist(2);
    K0_Twist(3) 0 -K0_Twist(1);
    -K0_Twist(2) K0_Twist(1) 0];
om0=K0_Twist*th0_Twist;
if det((eye(3)-R0)*K0_x+om0*K0_Twist')==0
    v0=T0d(1:3,4);
else
    v0=inv((eye(3)-R0)*K0_x+om0*K0_Twist')*d0;
end
% Final Twis
thf_Twist=atan2(0.5*norm([Rf(3,2)-Rf(2,3);Rf(1,3)-Rf(3,1);Rf(2,1)-Rf(1,2)]),0.5*(trace(Rf)-1));
Kf=(1/(2*sin(thf_Twist)))*[Rf(3,2)-Rf(2,3);Rf(1,3)-Rf(3,1);Rf(2,1)-Rf(1,2)];
Kf_x=[0 -Kf(3) Kf(2);
    Kf(3) 0 -Kf(1);
    -Kf(2) Kf(1) 0];
omf=Kf*thf_Twist;
vf=inv((eye(3)-Rf)*Kf_x+omf*Kf')*df;
% Interpolatting V
[a0_v_x,a1_v_x,a2_v_x,a3_v_x] =GenCoffs(0,tf,v0(1),vf(1));
[a0_v_y,a1_v_y,a2_v_y,a3_v_y] =GenCoffs(0,tf,v0(2),vf(2));
[a0_v_z,a1_v_z,a2_v_z,a3_v_z] =GenCoffs(0,tf,v0(3),vf(3));
%
for it=1:length(t)
    om_t_x=a0_om_x+a1_om_x*t(it)+a2_om_x*t(it).^2+a3_om_x*t(it).^3;
    om_t_y=a0_om_y+a1_om_y*t(it)+a2_om_y*t(it).^2+a3_om_y*t(it).^3;
    om_t_z=a0_om_z+a1_om_z*t(it)+a2_om_z*t(it).^2+a3_om_z*t(it).^3;
    om_t=[om_t_x;om_t_y;om_t_z];
    %
    % Interpolate Omega
    v_t_x=a0_v_x+a1_v_x*t(it)+a2_v_x*t(it).^2+a3_v_x*t(it).^3;
    v_t_y=a0_v_y+a1_v_y*t(it)+a2_v_y*t(it).^2+a3_v_y*t(it).^3;
    v_t_z=a0_v_z+a1_v_z*t(it)+a2_v_z*t(it).^2+a3_v_z*t(it).^3;
    v_t=[v_t_x;v_t_y;v_t_z];
    K=om_t/norm(om_t);
    K_x=[0 -K(3) K(2);
        K(3) 0 -K(1);
        -K(2) K(1) 0];
    %
    R_t=angleAxis2Rot(om_t);
    d_t=((eye(3)-R_t)*K_x+om_t*K')*v_t;
    %d_t=[dx(it);dy(it);dz(it)];
    T_t=[R_t,d_t;0 0 0 1];
    joints=IKfnc(T_t);
    Joint_Variable(it,:,4)=joints;
end
%% Plot
figure
for i=1:4
    for it=1:length(t)
        subplot(2,2,i)
        drawArm(Joint_Variable(it,1),Joint_Variable(it,2),Joint_Variable(it,3),Joint_Variable(it,4),joints(5),Joint_Variable(it,6), 1, Tfd)
        pause(0.5)
        grid
    end
end