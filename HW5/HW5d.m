num_matrices = 100;
t_06 = cell(1, num_matrices);
success = 0;
fail = 0;
for j= 1:num_matrices
    % Generate a random rotation matrix (3x3)
    R = rand(3, 3) * 2 - 1;  % Random values between -1 and 1
    [U, ~, V] = svd(R);  % Ensure orthogonal rotation matrix
    R = U * V';

    % Generate a random translation vector (3x1)
    translation = rand(3, 1) * 2 - 1;  % Random values between -1 and 1

    % Create the 4x4 transformation matrix
    t_06{j} = eye(4);  % Initialize as identity matrix
    t_06{j}(1:3, 1:3) = R;  % Set the rotation block
    t_06{j}(1:3, 4) = translation;  % Set the translation
    [theta1, theta2, theta3, theta4, theta5, theta6] = inverse_kinmatics(t_06{j});


    theta = [theta1,theta2, theta3, theta4, theta5, theta6];
    alpha = [pi/2, 0,-pi/2,pi/2,-pi/2,0];
    a = [0, 0.270, 0.070, 0, 0 ,0];
    d = [0.290, 0 ,0 , 0.302, 0 , 0.072];
    T = cell(1, 6);
    for i=1:6
        H = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
            sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
            0, sin(alpha(i)), cos(alpha(i)), d(i);
            0, 0, 0, 1];
        T{i} = H;
    end 
    T_06_fk = eye(4);
    for i=1:6
        T_06_fk = T_06_fk*T{i};
    end
t_06_j = t_06{j};    

t_norm = norm(t_06_j - T_06_fk, 'fro');
if t_norm < 7e-8
    success = success + 1;
else 
    fail = fail+1;
end 

end


