T_init = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
T_final = [0.5 -0.6124 0.6124 0.84515;
           0.6124 0.75 0.25 0.3251;
           -0.6124 0.25 0.75 -0.3251;
           0 0 0 1];

R_init = T_init(1:3,1:3);
R_final = T_final(1:3,1:3);

Omega_init = rot2AngleAxis(R_init);
Omega_final = rot2AngleAxis(R_final);

position_init = T_init(1:3,4);
position_final = T_final(1:3,4);

v_int = [Omega_init, position_init];

v_final = [Omega_final, position_final];

quiver3(position_init(1), position_init(2), position_init(3),T_init(1,1), T_init(2,1), T_init(3,1),"Color","red");
axis equal;
hold on;
quiver3(position_init(1), position_init(2), position_init(3),T_init(1,2), T_init(2,2), T_init(3,2),"Color","green");
quiver3(position_init(1), position_init(2), position_init(3),T_init(1,3), T_init(2,3), T_init(3,3),"Color","blue");

for i=1:10
    r=i/10;
    inter_v = v_int + (v_final - v_int)*r;
    position_d = inter_v(4:6);
    R = angleAxis2Rot(inter_v(1:3,1));
quiver3(position_d(1), position_d(2), position_d(3),R(1,1), R(2,1), R(3,1),"Color","red");
axis equal;
hold on;
quiver3(position_d(1), position_d(2), position_d(3),R(1,2), R(2,2), R(3,2),"Color","green");
quiver3(position_d(1), position_d(2), position_d(3),R(1,3), R(2,3), R(3,3),"Color","blue");
end

