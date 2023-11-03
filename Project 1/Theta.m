initial_pose = [1 0 0 1;
                0 0 -1 0.25;
                0 1 0 0.25;
                0 0 0 1];

final_pose = [0 -1 0 0.5;
              1 0 0 -0.5;
              0 0 1 0;
              0 0 0 1];


T = 2.0; % Total time duration

% Call the cubic interpolation function to get coefficients
theta_init =  abbInvKine(initial_pose,zeros(6,1));
disp(theta_init);
theta_final = abbInvKine(final_pose,theta_init(:,1));
disp(theta_final);