function [X, Y] = computeRecursiveDyn(P_ps, P_vs, P_pu, P_vu, N, x_hat_k, y_hat_k, U_k)

%  C = P_ps *  c0 + P_pu * U
% dC = P_vs * dc0 + P_vu * U

X = zeros(N, 2);
Y = zeros(N, 2);

X(:, 1) = P_ps * x_hat_k' + P_pu * U_k(1:N);
X(:, 2) = P_vs * x_hat_k' + P_vu * U_k(1:N);

Y(:, 1) = P_ps * y_hat_k' + P_pu * U_k(N+(1:N));
Y(:, 2) = P_vs * y_hat_k' + P_vu * U_k(N+(1:N));

end