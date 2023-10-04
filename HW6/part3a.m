initial_pose=[1 0 0 1;0 0 -1 0.25;0 1 0 0.25;0 0 0 1];
final_pose=[0 -1 0 0.5;1 0 0 -0.5;0 0 1 0;0 0 0 1];
%% Joint Spapce Interpolation
t0=0;
T=2;
t=linspace(t0,T,200);
initial_joints=inversekinematics(initial_pose,zeros(6,1));
%
final_joints=inversekinematics(final_pose,initial_joints(:,1));

[a0, a1, a2, a3] = cubiccoeficient(initial_joints,final_joints,T-t0);
theta_vals = cubicinterpolation(a0, a1, a2, a3, t0,t);
%% Draw Joint Space
out = VideoWriter('part3a');
out.open();
posList = zeros(3,200);
for i=1:length(t)

     T_tmp = forwardkinematics(theta_vals(1,i),theta_vals(2,i),theta_vals(3,i),theta_vals(4,i),theta_vals(5,i),theta_vals(6,i));
     posList(:,i) = T_tmp(1:3,4);
     drawArm(theta_vals(1,i),theta_vals(2,i),theta_vals(3,i),theta_vals(4,i),theta_vals(5,i),theta_vals(6,i),1,final_pose);
     pause(1/200);
end
out.close();

figure()
subplot(2,2,[2,4])
 plot(t,theta_vals/pi)
 legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6');
 xlabel('Time [s]')
 ylabel('Angle [rad/\pi]')
subplot(2,2,1)
 plot(tList,posList)
 legend('X','Y','Z')
 xlabel('Time [s]')
 ylabel('Position [m]')
subplot(2,2,3)
 plot3(posList(1,:),posList(2,:),posList(3,:))
 xlabel('X [m]')
 ylabel('Y [m]')
 zlabel('Z [m]')
 grid on;
 axis equal;

ax=axes('Units','Normal','Position',[.075 .125 .85 .85],'Visible','off');
set(get(ax,'Title'),'Visible','on')
title(titleText);
