function q = cubicSplineInterpolation(a0,a1 , a2, a3,t)

    q = a0 + a1 * t  + a2 * t^2 + a3 * t^3;
end
