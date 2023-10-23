% Load points for 'C' from points2D.mat


load('points2D.mat');

% Define desired starting position for 'C' and end position for 'M' in
% world frame
position_init_world = [-0.01; -0.3; 0.48];
position_final_world = [0.14;-0.3; 0.4];

%

position_init_points = [2.5; 4; 0];
position_final_points = [10; 0;0];
% Determine scaling factor for 'C' (adjust as needed)
scale_diff_world = position_final_world-position_init_world;
scale_diff_points = position_final_points-position_init_points;
magnitude_scale = norm(scale_diff_world)/norm(scale_diff_points);

rotation_matrix = rotX(pi/2);
scaled_points = magnitude_scale*[points_all, zeros(size(points_all, 1), 1)];

rotated_points_arr=[];
for i=1:26
    rotated_points = rotation_matrix*[scaled_points(i,1);scaled_points(i,2);scaled_points(i,3)];
    rotated_points_arr = [rotated_points_arr rotated_points(1:3)];
    
end
rotation_init = rotated_points_arr(:,1);
displacement_init =  position_init_world - rotation_init;

transformation_mat = [rotation_matrix, displacement_init;
    0 0 0 1];


transform_arr = [];
for i= 1:26
    transform = transformation_mat*[(scaled_points(i,:)),1]';
    transform_arr = [transform_arr transform(1:4)];
   
end
p = plot3(transform_arr(1,:),transform_arr(2,:),transform_arr(3,:), LineWidth=0.5, Color='k');

s = scatter3(transform_arr(1,:),transform_arr(2,:),transform_arr(3,:),"black");

scale = 0.01;
T_arr = [];
for i=1:25
    rot_x = rotX(pi/2);
    ang_z = angbetpoint(transform_arr(:,i),transform_arr(:,i+1));
    rot_z = rotZ(ang_z);
    R = rot_x*rot_z;
    T = [R transform_arr(1:3,i); 0 0 0 1];
    T_arr = [T_arr;T(1:4,1:4)];
    
end
[th1,th2,th3,th4,th5,th6,reachable] = abbInvKine(T_arr(1:4,:),zeros(6,1));
theta_init = [th1;th2;th3;th4;th5;th6];
theta_arr = [];
for i =1:24
    [th1,th2,th3,th4,th5,th6,reachable]=abbInvKine(T_arr((i*4)+1:(i*4)+4,:),theta_init(:,1));
    theta = [th1,th2,th3,th4,th5,th6];
    theta_arr = [theta_arr; theta];
end
 T_final = [R transform_arr(1:3,26); 0 0 0 1];

 
 


   
