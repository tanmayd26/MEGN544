t = 3.3219;
trajectory = [0	0	0;
1	0	0;
2	0.500000000000000	3;
3	0.200000000000000	0.500000000000000;
4	0.130000000000000	1;
5	0.130000000000000	1];

transPercent = 0.0284;

 % Check if t is before or after the trajectory
    if t <= trajectory(1, 1)
        t = trajectory(1, 1);
    elseif t >= trajectory(end, 1)
        t = trajectory(end, 1);
    end
    t_arr = trajectory(:,1);
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
    t_mid = trajectory(segmentIndex+1,1);
    t_end = trajectory(segmentIndex+2, 1);

    p_start = trajectory(segmentIndex, 2:end);
    p_mid = trajectory(segmentIndex+1,2:end);
    p_end = trajectory(segmentIndex+2, 2:end);

    
    % Calculate the time in the current segment
    % t_rel = (t - t_start) / (t_end - t_start);
    % disp(t_rel);
    % 
    % % Determine the shorter duration segment for interpolation
    % if t_rel <= transPercent
    %     t_short = transPercent * (t_end - t_start);
    % 
    %     t_long = t_end - t_start - t_short;
    % 
    % else
    %     t_long = (1 - transPercent) * (t_end - t_start);
    % 
    %     t_short = t_end - t_start - t_long;
    % end
    % 
    % disp(t_long);
    % disp(t_short);
    % 
    % % Calculate position, velocity, and acceleration
    % if t_rel <= transPercent
    %     % Constant acceleration phase
    %     v = (p_end - p_start) / t_short;
    %     p = p_start + v * (t_rel / transPercent);
    %     %v = (p_end - p_start) / t_short;
    %     a = zeros(size(p_start));
    % else
    %     % Linear phase
    %     p = p_start + (p_end - p_start) * (t_rel - transPercent) / (1 - transPercent);
    %     v = (p_end - p_start) / t_long;
    %     a = zeros(size(p_start));
    % end
    % disp(p);
    % disp(v);
    % disp(a);
    t_23 = transPercent * min(t_end-t_mid,t_mid-t_start);

    t_2 = t_mid-t_23;
    t_3 = t_mid + t_23;

    v_start_mid = (p_mid-p_start)/(t_mid-t_start);
    v_mid_end = (p_end-p_mid)/(t_end-t_mid);

    a_mid_end = (v_mid_end-v_start_mid)/(2*t_23);

    p_2 = p_mid-v_start_mid*t_23;

    for i=2:(size(trajectory)-1)
        if t>t_arr(i-1) && t<t_2
            pos() = p_start + v_start_mid(t-t_start);
        end
    end