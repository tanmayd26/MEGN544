% Define DH parameters for a single joint
a = 0;        % X length
alpha = pi/2;    % Angle around X (in radians)
d = 0;        % Z length
theta = pi/6;   % Angle around Z (in radians)


A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
     sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1]; % Create the transformation matrix


disp('T:');
disp(A); % Display the transformation matrix
