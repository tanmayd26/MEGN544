t = 3.3219;
trajectory = [0	0	0;
1	0	0;
2	0.500000000000000	3;
3	0.200000000000000	0.500000000000000;
4	0.130000000000000	1;
5	0.130000000000000	1];

transPercent = 0.0284;

if t <= trajectory(1, 1)
    t = trajectory(1, 1);
elseif t >= trajectory(end, 1)
    t = trajectory(end, 1);
end
%s = size(trajectory,1);
% Find the segment in the trajectory that contains t
for i = 2:size(trajectory, 1)
    if t <= trajectory(i, 1)
        segmentIndex = i - 1;
        break;
    end
end

% Extract relevant time and position data for the segment
t_start = trajectory(segmentIndex, 1);
disp(t_start);
t_end = trajectory(segmentIndex + 1, 1);
disp(t_end);
p_start = trajectory(segmentIndex, 2:end);
disp(p_start);
p_end = trajectory(segmentIndex + 1, 2:end);
disp(p_end);

% Calculate the time in the current segment
t_rel = (t - t_start) / (t_end - t_start);
disp(t_rel);

% Determine the shorter duration segment for interpolation
if t_rel <= transPercent
    t_short = transPercent * (t_end - t_start);

    t_long = t_end - t_start - t_short;

else
    t_long = (1 - transPercent) * (t_end - t_start);

    t_short = t_end - t_start - t_long;
end

disp(t_long);
disp(t_short);

% Calculate position, velocity, and acceleration
if t_rel <= transPercent
    % Constant acceleration phase
    % p = p_start + (p_end - p_start) * (t_rel / transPercent);
    % v = (p_end - p_start) / t_short;
    % a = zeros(size(p_start));
    
else
    % Linear phase
    p = p_start + (p_end - p_start) * (t_rel - transPercent) / (1 - transPercent);
    v = (p_end - p_start) / t_long;
    a = zeros(size(p_start));
end
disp(p);
disp(v);
disp(a);

