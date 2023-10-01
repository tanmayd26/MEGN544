function [ang] = angbetpoint(i,f)

z = f(3)-i(3);
x = f(1)-i(1);

ang = atan2(z,x);

end