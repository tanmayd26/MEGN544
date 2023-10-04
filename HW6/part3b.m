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

d_init = initial_pose(1:3,4);
d_final = final_pose(1:3,4);

pos_om_init = [d_init; om_init];
pos_om_final = [d_init; om_init];

initial_pose_inv = inversekinematics(initial_pose,zeros(6,1));
[a0, a1, a2, a3] = cubiccoeficient(pos_om_init,pos_om_final,T);
pos_all = cubicinterpolation(a0,a1,a2,a3,t0,t);
theta_vals = zeros(6,length(t));
for i=1:length(t)
    theta = norm(pos_all(4:6,i));
    k_vec = pos_all(4:6,i)/theta;
    Trans = [angleAxis2Rot(k_vec*theta) pos_all(1:3,i); 0 0 0 1];
    if i>1
        joint = inversekinematics(Trans,theta_vals(:,i-1));
    else
        joint = inversekinematics(Trans,[initial_pose_inv(6,1)]);
    end
        theta_vals(:,i) = transpose(joint);
        drawArm(theta_vals(1,i), theta_vals(2,i), theta_vals(3,i), theta_vals(4,i), theta_vals(5,i), theta_vals(6,i))
end


%
% out = VideoWriter('part3b');
% out.open();
% 
% for i=1:length(t)
% 
% 
% 
%      drawArm( T_tmp );
%      hold on;
% 
% % What Ever Else You Want To Plot...
% 
%     hold off;
%     out.writeVideo(getframe(H));
%     pause(1/N);
%     end
% end
% out.close();
