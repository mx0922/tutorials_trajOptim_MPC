function [A_d, B_d] = discrete_LIP_dynamics(delta_t, g, h)
% x(k+1) = A_d * x(k) + B_d * u(k)
%          ||            ||
%        output1       output2

w = sqrt(g / h);
temp = w*delta_t;
A_d = [cosh(temp), 1/w*sinh(temp);...
    w*sinh(temp), cosh(temp)];
B_d = [1-cosh(temp); -w*sinh(temp)];

end