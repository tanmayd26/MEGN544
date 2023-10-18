% constAccelInterp(t,trajectory,transPercent) % trajectory
% using constant acceleration approach.
% 
%  [p,v,a] = constAccelInterp(t,trajectory,transPercent) returns the
%  position, velocity and the acceleration at tiime t for a trajectory
%  interpolated using constant acceleration approach.
% p = position
% v = velocity
% a = acceleration
% t = time at which the position, velocity and acceleration is to be found
% trajectory = matrix with time and positions of the trajectory
% transPercent = signifies order of the trajectory i.e whether the trajectory is linear/quadratic.
% 

% Tanmay Desai
% 10922557
% MEGN 544A
% October 22nd 2023

function [p, v, a] = constAccelInterp(t, trajectory, transPercent)
    % Check if t is before or after the trajectory
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
    t_23 = transPercent * min()
end

