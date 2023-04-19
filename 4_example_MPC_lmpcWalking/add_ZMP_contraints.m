function [A, b] = add_ZMP_contraints(N, foot_length, foot_width, Z_ref_k, x_hat_k, y_hat_k)

A = zeros(4*N, 2*N);
b = zeros(4*N, 1);

foot_length_N = foot_length * ones(N, 1);
foot_width_N  = foot_width * ones(N, 1);

A(1:N, 1:N)     =  eye(N);
A(N+(1:N), 1:N) = -eye(N);

A(2*N+(1:N), N+(1:N)) =  eye(N);
A(3*N+(1:N), N+(1:N)) = -eye(N);

b(1:N)       =  Z_ref_k(:, 1) - 0.5 * foot_length_N;
b(N+(1:N))   = -Z_ref_k(:, 1) - 0.5 * foot_length_N;
b(2*N+(1:N)) =  Z_ref_k(:, 2) - 0.5 * foot_width_N;
b(3*N+(1:N)) = -Z_ref_k(:, 2) - 0.5 * foot_width_N;

end