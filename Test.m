% Define your initial and final ZYZ Euler angles (in radians) and positions
initial_ZYZ_angles = [0.1, 0.2, 0.3];  % ZYZ Euler angles (roll, pitch, yaw)
final_ZYZ_angles = [0.4, 0.5, 0.6];
initial_position = [1, 0, 0];  % Initial position [x, y, z]
final_position = [2, 0, 0];    % Final position

% Number of intermediate points (including the start and end points)
num_points = 5;

% Initialize a cell array to store intermediate transformations
T_intermediate = cell(1, num_points);

% Perform linear interpolation for ZYZ Euler angles and positions
for i = 1:num_points
    alpha = (i - 1) / (num_points - 1);
    interpolated_ZYZ_angles = (1 - alpha) * initial_ZYZ_angles + alpha * final_ZYZ_angles;
    interpolated_position = (1 - alpha) * initial_position + alpha * final_position;
    
    % Convert ZYZ Euler angles to a rotation matrix
    R = eul2rotm(interpolated_ZYZ_angles, 'zyz');
    
    % Create the homogeneous transformation matrix
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = interpolated_position;
    
    % Store the intermediate transformation
    T_intermediate{i} = T;
end

% Create a figure
figure;
hold on;

% Plot the intermediate points with orthogonal axes
for i = 1:num_points
    % Extract the translation vector
    translation = T_intermediate{i}(1:3, 4);
    
    % Extract the rotation matrix
    R = T_intermediate{i}(1:3, 1:3);
    
    % Compute the orthogonal axes (columns of the rotation matrix)
    x_axis = R(:, 1);
    y_axis = R(:, 2);
    z_axis = R(:, 3);
    
    % Plot the translation vector
    plot3(translation(1), translation(2), translation(3), 'ro'); % Red dot
    
    % Plot orthogonal axes using quiver3
    quiver3(translation(1), translation(2), translation(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2);
    quiver3(translation(1), translation(2), translation(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2);
    quiver3(translation(1), translation(2), translation(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2);
end

% Set axis labels and limits
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;

% Display the plot
hold off;
