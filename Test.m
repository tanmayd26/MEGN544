t = 3.97028338597540;
trajectory = [0	0	0;
    1	0	0;
    2	0.500000000000000	3;
    3	0.200000000000000	0.500000000000000;
    4	0.130000000000000	1;
    5	0.130000000000000	1];

transPercent = 0.042395942561634;

t_arr = trajectory(:,1);
% Find the segment in the trajectory that contains t
for i = 2:size(trajectory, 1)
    if t <= trajectory(i, 1)
        segmentIndex = i - 1;
        break;
    end
end

% Extract relevant time and position data for the segment
t_start = trajectory(segmentIndex, 1);
disp(t_start)
t_mid = trajectory(segmentIndex+1,1);
disp(t_mid);

if segmentIndex == size(trajectory, 1) - 1
    % This is the last segment of the trajectory
    t_end = trajectory(end, 1);  % Use the last time point as t_end
    disp(t_end);
    p_end = trajectory(end, 2:end);  % Use the last position data
    disp(p_end);
    % Handle the last segment as needed
else
    t_end = trajectory(segmentIndex + 2, 1);
    disp(t_end);
    p_end = trajectory(segmentIndex + 2, 2:end);
    disp(p_end);
    % Handle other segments as you were doing before
end

p_start = trajectory(segmentIndex, 2:end);
disp(p_start);
p_mid = trajectory(segmentIndex+1,2:end);
disp(p_mid);

t_23 = transPercent * min(t_end-t_mid,t_mid-t_start);
disp(t_23);

t_2 = t_mid-t_23;
disp(t_2);
t_3 = t_mid+t_23;
disp(t_3);

v_start_mid = (p_mid-p_start)/(t_mid-t_start);
disp(v_start_mid);
v_mid_end = (p_end-p_mid)/(t_end-t_mid);
disp(v_mid_end);

a_mid_end = (v_mid_end-v_start_mid)/(2*t_23);
disp(a_mid_end);

p_2 = p_mid-v_start_mid*t_23;
disp(p_2);

for i=2:(size(trajectory)-1)
    if t>t_arr(i-1) && t<t_2
        disp(i);
        pos = p_start + v_start_mid*(t-t_start);
        vel = v_start_mid;
        acc = zeros(size(pos));

    elseif t>t_2 && t<t_3
        pos = p_2 + v_start_mid*(t-t_2)+(1/2)*a_mid_end*(t-t_2)^2;
        vel = v_mid_end+a_mid_end*(t-t_2);
        acc = a_mid_end;

    elseif segmentIndex == (size(trajectory)-1) && t<t_arr(i-1) && t>t_2
        pos = p_start;
        vel = zeros(size(pos));
        acc = zeros(size(pos));

    end
    
end
disp(pos);
    disp(vel);
    disp(acc);