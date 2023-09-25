T_init = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
T_final = [0.5 -0.6124 0.6124 0.84515;
           0.6124 0.75 0.25 0.3251;
           -0.6124 0.25 0.75 -0.3251;
           0 0 0 1]; 

%Let's create a vector with angles and vector positions.

theta_init = atan2(sqrt(0^2+0^2),1);
phi_init = atan2(0,0);
psi_init = atan2(0,0);
position_init = [0 0 0];

theta_final = atan2(sqrt((0.6124)^2+(0.25)^2),0.75);
phi_final = atan2(0.25/sin(theta_final),0.6214/sin(theta_final));
psi_final = atan2(0.25/sin(theta_final),0.6214/sin(theta_final));
position_final = [0.8415 0.3251 -0.3251];

v_int = [phi_init theta_init psi_init, position_init];

v_final = [phi_final theta_final psi_final, position_final];

quiver3(position_init(1), position_init(2), position_init(3),T_init(1,1), T_init(2,1), T_init(3,1),"Color","red");
axis equal;
hold on;
quiver3(position_init(1), position_init(2), position_init(3),T_init(1,2), T_init(2,2), T_init(3,2),"Color","green");
quiver3(position_init(1), position_init(2), position_init(3),T_init(1,3), T_init(2,3), T_init(3,3),"Color","blue");
for i=1:10
    r = i/10;
 inter_v = v_int + (v_final - v_int)*r;

inter_phi = inter_v(1);
inter_theta = inter_v(2);
inter_psi = inter_v(3);

R = rprRot(-inter_phi,inter_theta);

position_d = inter_v(4:6);
quiver3(position_d(1), position_d(2), position_d(3),R(1,1), R(2,1), R(3,1),"Color","red");
axis equal;
hold on;
quiver3(position_d(1), position_d(2), position_d(3),R(1,2), R(2,2), R(3,2),"Color","green");
quiver3(position_d(1), position_d(2), position_d(3),R(1,3), R(2,3), R(3,3),"Color","blue");
end 
