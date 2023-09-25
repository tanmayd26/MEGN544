% Define DH parameters for a single joint
a = 1;        % Link length
alpha = 0;    % Link twist (in radians)
d = 1;        % Link offset
theta = 0;   % Joint angle (in radians)

% Create the transformation matrix
A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
     sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1];

% Display the transformation matrix
disp('Transformation Matrix:');
disp(A);
