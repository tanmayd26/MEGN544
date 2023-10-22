t = 2.09146304795145;
trajectory = [0	0	0;
    1	0	0;
    2	0.500000000000000	3;
    3	0.200000000000000	0.500000000000000;
    4	0.130000000000000	1;
    5	0.130000000000000	1];

transPercent = 0.499731229595412;

% Check if t is before or after the trajectory
if t <= trajectory(1, 1)
    t = trajectory(1, 1);
elseif t >= trajectory(end, 1)
    t = trajectory(end, 1);
end
if t>=trajectory(size(trajectory,1),1)
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