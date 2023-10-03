function q = cubicSplineInterpolation(q0, qf, T, t)
    a0 = q0;
    a1 = 0;
    a2 = (3 / T^2) * (qf - q0);
    a3 = (-2 / T^3) * (qf - q0);
    
    q = a0 + a1 * (t - 0) + a2 * (t - 0).^2 + a3 * (t - 0).^3;
end