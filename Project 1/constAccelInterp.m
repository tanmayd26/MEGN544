% constAccelInterp(t,trajectory,transPercent) 
%
%  [p,v,a] = constAccelInterp(t,trajectory,transPercent) returns the
%  position, velocity and the acceleration at time t for a trajectory
%  interpolated using constant acceleration approach.
% output1 = position
% output2 = velocity
% output3 = acceleration
% 
% input1 = time at which the position, velocity and acceleration is to be found
% input2 = matrix with time and positions of the trajectory
% input3 = signifies order of the trajectory i.e whether the trajectory is in const vel/ const acc segment.
%
% Tanmay Desai
% 10922557
% MEGN 544A
% October 22nd 2023

function [p, v, a] = constAccelInterp(t, trajectory, transPercent)
% Check if t is before or after the trajectory

if t>=trajectory(end,1)
    for i = 1:2
        p(i) = trajectory(size(trajectory,1),i+1);
        v(i) = 0;
        a(i) = 0;
    end
elseif t<=trajectory(1,1)
    for i = 1:2
        p(i) = trajectory(1,i+1);
        v(i) = 0;
        a(i) = 0;
    end
else
    for i =1:size(trajectory,1)
        if t >trajectory(i,1) && t<trajectory(i+1,1)
            ta = trajectory(i,1);
            tb = trajectory(i+1, 1);
            if i == size(trajectory,1) - 1
                tc = tb;
                to = trajectory(i-1,1);
            elseif i == 1
                tc = trajectory(i+2,1);
                to = ta;
            else
                tc = trajectory(i+2,1);
                to = trajectory(i-1,1);
            end
            for j = 1:2
                if i == size(trajectory,1) - 1
                    thetaC(j) = 0;
                else
                    thetaC(j) = trajectory(i+2, 1+j);
                end
                thetaA(j) = trajectory(i, 1+j);
                thetaB(j) = trajectory(i+1, 1+j);

            end
            break;
        end
    end

    T23 = transPercent* min(tc-tb,tb-ta);
    T12 = transPercent * min(tb-ta,ta-to);

    t1 = ta + T12;
    if t<t1
        ta = trajectory(i-1,1);
        tb = trajectory(i, 1);
        tc = trajectory(i+1,1);
        to = trajectory(i-2,1);

        for j = 1:2
            thetaA(j) = trajectory(i-1, 1+j);
            thetaB(j) = trajectory(i, 1+j);
            thetaC(j) = trajectory(i+1, 1+j);
        end

        T23 = transPercent* min(tc-tb,tb-ta);
        T12 = transPercent * min(tb-ta,ta-to);
    end


    t2 = tb - T23;
    t3 = tb + T23;

    for i = 1:2
        Vab(i) = (thetaB(i) - thetaA(i)) / (tb - ta);
        Vbc(i) = (thetaC(i) - thetaB(i)) / (tc - tb);
        a23(i) = (Vbc(i) - Vab(i))/(2*T23);
        P2(i) = thetaB(i) - Vab(i) * T23;

        if t>t1 && t<t2
            p(i) = thetaA(i) + Vab(i)*(t-ta);
            v(i) = Vab(i);
            a(i) = 0;
        elseif t>t2 && t<t3
            p(i) = P2(i) + Vab(i)*(t - t2) + 0.5*a23(i)*(t - t2)^2;
            v(i) = Vab(i) + a23(i)*(t - t2);
            a(i) = a23(i);

        end


    end

end