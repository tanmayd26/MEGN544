T_01 = [ 1.0000         0         0    1.0000;
         0    0.0000   -1.0000         0;
         0    1.0000    0.0000         0;
         0         0         0    1.0000];

T_12 = [   0.0000   -0.0000    1.0000    0.0000
    1.0000    0.0000   -0.0000    1.0000
         0    1.0000    0.0000         0
         0         0         0    1.0000];

T_23 = [1     0     0     1;
     0     1     0     0;
     0     0     1     1;
     0     0     0     1];

T_03 = T_01*T_12*T_23;