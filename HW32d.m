q = [0 0.3535 0.3535];
q_norm = norm(q);

k = q/q_norm;

theta = 1.0470;

omega = theta * k;

d = [0.8415 0.3251 -0.3251];

k_x = [0 -0.7071 0.7071;
    0.7071 0 0;
    -0.7071 0 0];

v = (((sin(theta))/2*(1-cos(theta)))*eye(3));...
    %+(2*(1-cos(theta))-theta*sin(theta))/(2*theta*(1-cos(theta)))*k*transpose(k)-1/2*(k_x))*d;