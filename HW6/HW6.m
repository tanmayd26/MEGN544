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
coefficients = cubicInterpolation(initial_pose, final_pose, T, @inversekinematics, @forwardkinematics);