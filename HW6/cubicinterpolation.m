function q = cubicinterpolation(a0,a1,a2,a3,t0,T)
t = reshape(T,1,numel(T));
    q = a0 + a1*(t-t0) + a2*(t-t0).^2 + a3*(t-t0).^3;
end

