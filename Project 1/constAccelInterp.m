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
% if t <= trajectory(1, 1)
%     t = trajectory(1, 1);
% elseif t >= trajectory(end, 1)
%     t = trajectory(end, 1);
% end
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
disp(t_start)
t_mid = trajectory(segmentIndex+1,1);
disp(t_mid);
% t_end = trajectory(segmentIndex+2, 1);



if segmentIndex == size(trajectory, 1) - 1
    % This is the last segment of the trajectory
    t_end = trajectory(end, 1);  % Use the last time point as t_end
    disp(t_end);
    p_end = trajectory(end, 2:end);  % Use the last position data
    % Handle the last segment as needed
else
    t_end = trajectory(segmentIndex + 2, 1);
    p_end = trajectory(segmentIndex + 2, 2:end);
    % Handle other segments as you were doing before
end

p_start = trajectory(segmentIndex, 2:end);
disp(p_start);
p_mid = trajectory(segmentIndex+1,2:end);
disp(p_mid);
% p_end = trajectory(segmentIndex+2, 2:end);
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
    p = pos;
    v = vel;
    a = acc;

end