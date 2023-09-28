% Load points for 'C' from points2D.mat
load('points2D.mat');

% Define desired starting position for 'C' and end position for 'M' in
% world frame
position_init_world = [-0.01; -0.3; 0.48];
position_final_world = [0.14;-0.3; 0.4];

%
position_init_points = [0, 2.5, 0];
position_final_points = [10, 0, 0];
% Determine scaling factor for 'C' (adjust as needed)
scale_diff_world = position_final_world-position_init_world;
scale_diff_points = position_final_points-position_init_points;
magnitude_scale = norm(scale_diff_points)/norm(scale_diff_world);


for i= 1:26
    scaled_points = magnitude_scale*[points_all(1:i,1), points_all(1:i,2)];
end

rotation_matrix = rotX(pi/2);
translation_vec = [scaled_points(1,1);scaled_points(1,2);0];

T_init = [rotation_matrix, translation_vec;
    0 0 0 1];
quiver3(translation_vec(1), translation_vec(2), translation_vec(3),T_init(1,1), T_init(2,1), T_init(3,1),"Color","red");
axis equal;
hold on;
quiver3(translation_vec(1), translation_vec(2), translation_vec(3),T_init(1,2), T_init(2,2), T_init(3,2),"Color","green");
quiver3(translation_vec(1), translation_vec(2), translation_vec(3),T_init(1,3), T_init(2,3), T_init(3,3),"Color","blue");
for i= 1:26
    T_init = T_init.*[scaled_points(i,1);scaled_points(i,2);0];
 position_d = T_init(1:3,4);
 R = T_init(1:3,1:3);
quiver3(position_d(1), position_d(2), position_d(3),R(1,1), R(2,1), R(3,1),"Color","red");
axis equal;
hold on;
quiver3(position_d(1), position_d(2), position_d(3),R(1,2), R(2,2), R(3,2),"Color","green");
quiver3(position_d(1), position_d(2), position_d(3),R(1,3), R(2,3), R(3,3),"Color","blue");
end

