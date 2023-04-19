function [Q, p_k] = compute_objective_terms(alpha, beta, gamma, ...
        step_duration, no_steps_per_T, N, stride_length, stride_width, ...
        P_ps, P_pu, P_vs, P_vu, x_hat_k, y_hat_k, Z_ref_k)
% 将初始设置的目标函数转换成QP求解器的标准形式
% 即得到：0.5*x'*Q*x + f'x 中对应的的Q和f 

p_k = zeros(2*N, 1);

Q_prime = alpha * eye(N) + beta * (P_pu'*P_pu) + gamma * (P_vu'*P_vu);
Q = kron(eye(2), Q_prime);

x_r_N = stride_length / no_steps_per_T * ones(N, 1);
y_r_N =  stride_width / no_steps_per_T * ones(N, 1);

x_dotr_N = stride_length / step_duration * ones(N, 1);
y_dotr_N =  stride_width / step_duration * ones(N, 1);

p_k(1:N) = gamma * (P_vu'*(P_vs*x_hat_k')-P_vu'*x_dotr_N) + ...
    beta * (P_pu'*(P_ps*x_hat_k')-P_pu'*x_r_N) - alpha*Z_ref_k(:, 1);

p_k(N+(1:N)) = gamma * (P_vu'*(P_vs*y_hat_k')-P_vu'*y_dotr_N) + ...
    beta * (P_pu'*(P_ps*y_hat_k')-P_pu'*y_r_N) - alpha*Z_ref_k(:, 2);    
    
end