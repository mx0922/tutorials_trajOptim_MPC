function [P_ps, P_vs, P_pu, P_vu] = computeRecursiveMat(delta_t, g, h, N)
% This function is very important, good reference.
% outputs:
% P_ps -- size: N*2
% P_vs -- size: N*2
% P_pu -- size: N*N
% P_vu -- size: N*N

[A_d, B_d] = discrete_LIP_dynamics(delta_t, g, h);

P_ps = zeros(N, 2);
P_vs = zeros(N, 2);
temp_pu = zeros(N, 1);
temp_vu = zeros(N, 1);

for i = 1:N
    A_d_pow = A_d^i;
    P_ps(i, :) = A_d_pow(1, :);
    P_vs(i, :) = A_d_pow(2, :);
    temp_u = A_d^(i-1) * B_d;
    temp_pu(i) = temp_u(1);
    temp_vu(i) = temp_u(2);    
end

P_pu = zeros(N, N);
P_vu = zeros(N, N);
for i = 1:N
    P_pu(i, 1:i) = flip(temp_pu(1:i));
    P_vu(i, 1:i) = flip(temp_vu(1:i));
end

end