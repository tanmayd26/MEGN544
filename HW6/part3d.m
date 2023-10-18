initial_pose=[1 0 0 1;0 0 -1 0.25;0 1 0 0.25;0 0 0 1];
final_pose=[0 -1 0 0.5;1 0 0 -0.5;0 0 1 0;0 0 0 1];
t0=0;
T=2;
t=linspace(t0,T,200);

t_init = transform2Twist(initial_pose);
t_final = transform2Twist(final_pose);

%This step is for getting the initial parameters as this is input to
%calculate 
initial_pose_inv = inversekinematics(initial_pose,zeros(6,1));
[a0, a1, a2, a3] = cubiccoeficient(t_init,t_final,T);
pos_all = cubicinterpolation(a0,a1,a2,a3,t0,t);
d_vals = zeros(3,length(t));
theta_vals = zeros(6,length(t));
out = VideoWriter('part3d');
out.open();
for i=1:length(t)
    % This step gives us the transformation matrix
    tr = twist2Transform(pos_all(:,i));
    if i>1
        joint = inversekinematics(tr,theta_vals(:,i-1));
    else
        joint = inversekinematics(tr,initial_pose_inv(:,1));
    end
        theta_vals(:,i) = joint';
        trans = forwardkinematics(theta_vals(1,1),theta_vals(2,1),theta_vals(3,1), theta_vals(4,1),theta_vals(5,1),theta_vals(6,1));
        position = trans(1:3,4);
       drawArm(theta_vals(1,i), theta_vals(2,i), theta_vals(3,i), theta_vals(4,i), theta_vals(5,i), theta_vals(6,i),1,final_pose);
        pause(1/200);
end
out.close();

figure(1)

 plot(t,theta_vals/pi)
 legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6');
 xlabel('Time [s]')
 ylabel('Angle [rad/\pi]')
 grid on;
 axis equal;

figure(2)
 plot(t,position(1:3,:))
 legend('X','Y','Z')
 xlabel('Time [s]')
 ylabel('Position [m]')
    grid on;
 axis equal;

figure(3)
 plot3(position(1,:),position(2,:),position(3,:))
 xlabel('X [m]')
 ylabel('Y [m]')
 zlabel('Z [m]')
 grid on;
 axis equal;

