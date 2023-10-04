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
 theta_init =  inversekinematics(initial_pose,zeros(6,1));

theta_final = inversekinematics(final_pose,theta_init(:,1));

t = 0:0.01:T;
coeff = [];
for i =1:6
    
    [a0, a1, a2, a3] = cubiccoeficient(theta_init(i),theta_final(i),T);
    coeff = [coeff [a0; a1; a2; a3]];
end

for i=1:6

    q = cubicinterpolation(coeff(1,i),coeff(2,i),coeff(3,i),coeff(4,i),0,t);

end

