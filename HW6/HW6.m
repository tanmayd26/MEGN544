initial_pose = [1 0 0 1;
    0 0 -1 0.25;
    0 1 0 0.25;
    0 0 0 1];

final_pose = [0 -1 0 0.5;
    1 0 0 -0.5;
    0 0 1 0;
    0 0 0 1];

T = 2;

a1 = 1;
a2 = 0.5;

coefficient = cubicInterpolation(initial_pose,final_pose,T,a1,a2);
