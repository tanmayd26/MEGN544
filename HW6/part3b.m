initial_pose=[1 0 0 1;0 0 -1 0.25;0 1 0 0.25;0 0 0 1];
final_pose=[0 -1 0 0.5;1 0 0 -0.5;0 0 1 0;0 0 0 1];
t0=0;
T=2;
t=linspace(t0,T,200);
%% Chartesian/Angle-Axis
R_init=initial_pose(1:3,1:3);
R_final=final_pose(1:3,1:3);
om_init=rot2AngleAxis(R_init);
%
om_final=rot2AngleAxis(R_final);
% Interpolatting Omega
[a0_om_x,a1_om_x,a2_om_x,a3_om_x] =cubiccoeficient(om_init(1),om_final(1),T);
[a0_om_y,a1_om_y,a2_om_y,a3_om_y] =cubiccoeficient(om_init(2),om_final(2),T);
[a0_om_z,a1_om_z,a2_om_z,a3_om_z] =cubiccoeficient(om_init(3),om_final(3),T);
% Interpolating position
d0=initial_pose(1:3,4);
df=final_pose(1:3,4);
% Interpolate Displacement
dx=linspace(d0(1),df(1),length(t));
dy=linspace(d0(2),df(2),length(t));
dz=linspace(d0(3),df(3),length(t));
% Interpolate Omega
for j=1:length(t)
    om_t_x=a0_om_x+a1_om_x*t(j)+a2_om_x*t(j).^2+a3_om_x*t(j).^3;
    om_t_y=a0_om_y+a1_om_y*t(j)+a2_om_y*t(j).^2+a3_om_y*t(j).^3;
    om_t_z=a0_om_z+a1_om_z*t(j)+a2_om_z*t(j).^2+a3_om_z*t(j).^3;
    om_t=[om_t_x;om_t_y;om_t_z];
    %
    R_t=angleAxis2Rot(om_t);
    d_t=[dx(j);dy(j);dz(j)];
    T_t=[R_t,d_t;0 0 0 1];
    joints=inversekinematics(T_t,);
    Joint_Variable(j,:,2)=joints;
end

%
out = VideoWriter('part3b');
out.open();

for i=1:length(t)
    for j=1:4

     T_tmp = Joint_Variable(i,:,j);
     drawArm( T_tmp );
     hold on;

% What Ever Else You Want To Plot...

    hold off;
    out.writeVideo(getframe(H));
    pause(1/N);
    end
end
out.close();
