clc
clear all
close all
T0d=[1 0 0 1;0 0 -1 0.25;0 1 0 0.25;0 0 0 1];
Tfd=[0 -1 0 0.5;1 0 0 -0.5;0 0 1 0;0 0 0 1];
%% Joint Spapce Interpolation
t0=0;
tf=2;
t=linspace(t0,tf,8);
j0=IKfnc(T0d);
%
jf=IKfnc(Tfd);
%% Draw Joint Space
A=zeros(6,4); % Matrix to Store the Interpolation Cofficients for all joints
Joint_Variable=zeros(length(t),6,4);
for it=1:length(t)
        joints=zeros(1,6);
    for i=1:6 % Outer Loop for eact Joint
        [a0, a1,a2,a3] =GenCoffs(0,tf,j0(i),jf(i));
        joints(i)=a0+a1*t(it)+a2*t(it).^2+a3*t(it).^3;
        Joint_Variable(it,i,1)=joints(i);
    end
   
end
%% Chartesian/Angle-Axis
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
% Interpolate Displacement
dx=linspace(d0(1),df(1),length(t));
dy=linspace(d0(2),df(2),length(t));
dz=linspace(d0(3),df(3),length(t));
% Interpolate Omega
for it=1:length(t)
    om_t_x=a0_om_x+a1_om_x*t(it)+a2_om_x*t(it).^2+a3_om_x*t(it).^3;
    om_t_y=a0_om_y+a1_om_y*t(it)+a2_om_y*t(it).^2+a3_om_y*t(it).^3;
    om_t_z=a0_om_z+a1_om_z*t(it)+a2_om_z*t(it).^2+a3_om_z*t(it).^3;
    om_t=[om_t_x;om_t_y;om_t_z];
    %
    R_t=angleAxis2Rot(om_t);
    d_t=[dx(it);dy(it);dz(it)];
    T_t=[R_t,d_t;0 0 0 1];
    joints=IKfnc(T_t);
    Joint_Variable(it,:,2)=joints;
end
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
%
out = VideoWriter('HW6.');
out.open();
H = figure(3);
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
%% IK
function joints=IKfnc(Td)
    l1=1; l2=0.5;
    a = [l1,l2,0,0,0,0];
    alpha = [0 0 0 pi/2 -pi/2 0];
   
    th4=zeros(1,4);
    th6=zeros(1,2);
    P=Td(1:3,4);
    R = Td(1:3,1:3);
    for i=1:2
        px=P(1);
        py=P(2);
        %
        N=(l1+l2)^2-(px^2+py^2);
        D=(px^2+py^2)-(l1-l2)^2;
        %
        th2 = [2*atan(sqrt(N/D)), -2*atan(sqrt(N/D))];
        th2 =real(th2);
        th1 = (atan2(py,px))-atan2((l2*sin(th2)),l1+(l2*cos(th2)));
        %
        th5 = [atan2(sqrt((R(1,3))^2+(R(2,3))^2),R(3,3)),-atan2(sqrt((R(1,3))^2+(R(2,3))^2),R(3,3))];
        %
        for k=1:2
            if sin(th5(k))==0
                th6(k)=pi/2;
            else
           
            th6(k) = atan2(-(R(3,2))/sin(th5(k)),(R(3,1))/sin(th5(k)));
            end
        end
        n=1;
        for k=1:2
            for l=1:2
                if sin(th5(k))==0
                    th5k=pi/2;
                    th4(n) = atan2(-(R(2,3))/sin(th5k),-(R(1,3))/sin(th5k))-th1(l)-th2(l);
                else
                    th4(n) = atan2(-(R(2,3))/sin(th5(k)),-(R(1,3))/sin(th5(k)))-th1(l)-th2(l);
                end
                    n=n+1;
            end
        end
        %%
        for k=1:2
            if sin(th5(k))==0
                th6(k)=pi/2;
            else
           
            th6(k) = atan2(-(R(3,2))/sin(th5(k)),(R(3,1))/sin(th5(k)));
            end
        end
        %%
        d3=P(3);
 
    end
    %% FW
    d=[0,0,d3,0,0,0];
    n=1;
    for k=1:2
        for l=1:2
            th=[th1(1,k),th2(1,k),0,th4(1,l),th5(1,k),th6(1,k)];
            T=eye(4,4);
            for j=1:6
                T=T*dhTransform(a(j),d(j),alpha(j),th(j));            
            end
             if norm(round(T,4)-round(Td,4))==0
                    break
             end
            n=n+1;      
        end
         if norm(round(T,4)-round(Td,4))==0
            break
         end
    end
    joints=th;
    joints(3)=d3;
end
%% DH Table
function H = dhTransform(a, d, alpha, theta)
H=[
        cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
        sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0, sin(alpha), cos(alpha), d;
        0, 0, 0, 1
    ];
     
end
function [a0,a1,a2,a3]=GenCoffs(t0,tf,q0,qf)
    M =[1 t0 t0^2 t0^3;
        1 tf tf^2 tf^3;
        0 1 2*t0 3*t0^2;
        0 1 2*tf 3*tf^2];
    a = inv(M)*[q0;qf;0;0];
    a0 = a(1);
    a1 = a(2);
    a2 = a(3);
    a3 = a(4);
end
function [q]=Interpol(a0,a1,a2,a3,t0,t)
    t=[t0 t];
    q=zeros(1,length(t));
    for i=1:length(q)
        q(i)=a0+a1*t(i)+a2*t(i)^2+a3*t(i)^3;
    end
   
end