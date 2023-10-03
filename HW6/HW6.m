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

[theta1_init, theta2_init, d3_init, theta4_init, theta5_init, theta6_init] = inversekinematics(initial_pose);
theta_init = [theta1_init, theta2_init, d3_init, theta4_init, theta5_init, theta6_init];
[theta1_final, theta2_final, d3_final, theta4_final, theta5_final, theta6_final] = inversekinematics(final_pose);
theta_final = [theta1_final, theta2_final, d3_final, theta4_final, theta5_final, theta6_final];

coeff = [];
for i =1:6
    
    [a0, a1, a2, a3] = cubiccoeficient(theta_init(i),theta_final(i),T);
    coeff = [coeff [a0; a1; a2; a3]];
end

for i = 1:6
    
end