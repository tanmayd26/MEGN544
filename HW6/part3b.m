initial_pose=[1 0 0 1;0 0 -1 0.25;0 1 0 0.25;0 0 0 1];
final_pose=[0 -1 0 0.5;1 0 0 -0.5;0 0 1 0;0 0 0 1];
t0=0;
T=2;
t=linspace(t0,T,200);

R_init=initial_pose(1:3,1:3);
R_final=final_pose(1:3,1:3);
om_init=rot2AngleAxis(R_init);
%
om_final=rot2AngleAxis(R_final);

d_init = initial_pose(1:3,4);
d_final = final_pose(1:3,4);

pos_om_init = [d_init; om_init];
pos_om_final = [d_final; om_final];

initial_pose_inv = inversekinematics(initial_pose,zeros(6,1));
[a0, a1, a2, a3] = cubiccoeficient(pos_om_init,pos_om_final,T);
pos_all = cubicinterpolation(a0,a1,a2,a3,t0,t);
theta_vals = zeros(6,length(t));
out = VideoWriter('part3b');
out.open();
for i=1:length(t)
    theta = norm(pos_all(4:6,i));
    k_vec = pos_all(4:6,i)/theta;
    Trans = [angleAxis2Rot(k_vec*theta) pos_all(1:3,i); 0 0 0 1];
    if i>1
        joint = inversekinematics(Trans,theta_vals(:,i-1));
    else
        joint = inversekinematics(Trans,initial_pose_inv(:,1));
    end
        theta_vals(:,i) = joint';
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
 plot(t,pos_all(1:3,:))
 legend('X','Y','Z')
 xlabel('Time [s]')
 ylabel('Position [m]')
    grid on;
 axis equal;

figure(3)
 plot3(pos_all(1,:),pos_all(2,:),pos_all(3,:))
 xlabel('X [m]')
 ylabel('Y [m]')
 zlabel('Z [m]')
 grid on;
 axis equal;

