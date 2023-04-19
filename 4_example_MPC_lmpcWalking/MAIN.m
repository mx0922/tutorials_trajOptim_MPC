% Linear MPC for walking pattern generation using LIPM. Refer to the P. B.
% Wieber's paper(Humanoids06) and the Chapter 48 of "Handbook of Robotics".
%
% Written by Xiang Meng (√œœÈ) in BIT on 2023/3/5.
%
% The python code can be found at:
% https://github.com/machines-in-motion/lmpc_walking
% 

% Linear MPC for optimizing the CoP.
clear; close all; clc

flag_movie = 1;      % walking video record: 1 - yes; 0 - no

%% LIPM params
h = 0.65;            % constant height (m, meter)
g = 9.81;            % gravity
foot_length = 0.2;   % foot length (for CoP contraints)(m)
foot_width  = 0.1;   % foot width (m)
ankle_height = 0.07; % ankle height (m)

%% weights of objective function
alpha = 0.1;    % CoP error
beta  = 0.001;  % CoM position error
gamma = 0.001;  % CoM velocity error

%% MPC params
delta_t = 0.1;                              % sampling time interval
step_time = 0.8;                            % step duration
no_steps_per_T = round(step_time/delta_t);
N = 16;                                     % preceding horizon

%% walking params (User defined parameters)
flag_beginSwgFoot = 'r';                                    % 'r' or 'l', default: 'r'
step_length = 0.30;                                         % fixed step length (m)
half_step_width = 0.08;                                     % half of step width (m)
step_width = 2 * half_step_width;                           % step width (m)
no_desired_steps = 12;                                      % number of desired walking steps   
no_planned_steps = 2+no_desired_steps;                      % planning 2 steps ahead
desired_walking_time = no_desired_steps * no_steps_per_T;
planned_walking_time = no_planned_steps * no_steps_per_T;

%% CoM initial state°¢ Footholds °¢ ref CoP 
%%%%%%%%%   CoM initial state: [x, xdot]'   %%%%%%%%%%%%%%%
x_hat_0 = [ 0, 0];
y_hat_0 = [ 0, 0];

%%%%%%%%%  Footholds' position£∫ [x, y]  %%%%%%%%%%%%%%
foot_step_0 = [0, 0];

% desired footholds setting
desiredFoot_steps = zeros(no_desired_steps, 2);
for i = 1:no_desired_steps
    if i == 1 || i == 2   % first two steps to be ready to walk
        desiredFoot_steps(i, :) = foot_step_0;
    elseif i == 3         % default: right foot first move
        if flag_beginSwgFoot == 'r' % right foot
            desiredFoot_steps(i, :) = [0,  half_step_width];
        else % left foot
            desiredFoot_steps(i, :) = [0, -half_step_width];
        end
    elseif i == no_desired_steps || i == no_desired_steps-1 % last two steps to stop
        desiredFoot_steps(i, :) = [desiredFoot_steps(i-1, 1), 0];
    else % x direction: add a step_length; y direction: alternating position and negative
        desiredFoot_steps(i, 1) =  desiredFoot_steps(i-1, 1) + step_length;
        desiredFoot_steps(i, 2) = -desiredFoot_steps(i-1, 2);
    end
end

% desired CoP of each time interval
desired_Z_ref = zeros(desired_walking_time, 2);
j = 0;
for i = 1:no_desired_steps
    desired_Z_ref(j+(1:no_steps_per_T), :) = desiredFoot_steps(i, :) .* ones(no_steps_per_T, 1);
    j = j + no_steps_per_T;
end

planned_Z_ref = zeros(planned_walking_time, 2);
planned_Z_ref(1:desired_walking_time, :) = desired_Z_ref;
planned_Z_ref(desired_walking_time+1:planned_walking_time, :) = desired_Z_ref(end, :) .* ones(planned_walking_time-desired_walking_time, 1);

% planned feet trajectory
generate_feet_traj;

%% MPC loop
x_hat_k = x_hat_0;
y_hat_k = y_hat_0;
Z_ref_k = planned_Z_ref(1:N, :);

X_k   = zeros(N, 2);
Y_k   = zeros(N, 2);
Z_x_k = zeros(N, 1);
Z_y_k = zeros(N, 1);

X_total   = zeros(desired_walking_time+1, 2);
Y_total   = zeros(desired_walking_time+1, 2);
Z_x_total = zeros(desired_walking_time, 1);
Z_y_total = zeros(desired_walking_time, 1);

T_k = 0;

% Key Line!!!
[P_ps, P_vs, P_pu, P_vu] = computeRecursiveMat(delta_t, g, h, N);

herizon_data = cell(1, desired_walking_time);

X_total(1, :) = x_hat_0;
Y_total(1, :) = y_hat_0;

for i = 1:desired_walking_time
    time_k = T_k + delta_t * (1:N)';
    
    herizon_data{i}.zmp_ref = Z_ref_k;
    
    % min_x  0.5*x'*Q*x + f'*x
    % s.t.   A*x <= b; Aeq*x = beq; lb <= x <= ub 
    [Q, f] = compute_objective_terms(alpha, beta, gamma, ...
        step_time, no_steps_per_T, N, step_length, step_width, ...
        P_ps, P_pu, P_vs, P_vu, x_hat_k, y_hat_k, Z_ref_k);
    
    % A_zmp * x >= b_zmp
    [A_zmp, b_zmp] = add_ZMP_contraints(N, foot_length, foot_width, Z_ref_k, x_hat_k, y_hat_k);
    
    % using QP to solve: remember to add a 'minus(-)' to the A_zmp and b_zmp
    current_U = quadprog(Q, f, -A_zmp, -b_zmp);
    
    % calculate the state using the optimized CoP
    [X_k, Y_k] = computeRecursiveDyn(P_ps, P_vs, P_pu, P_vu, N, x_hat_k, y_hat_k, current_U);
    
    herizon_data{i}.X_k = X_k;
    herizon_data{i}.Y_k = Y_k;
    herizon_data{i}.time_k = time_k;
    
    % pick the first CoM state
    X_total(i+1, :) = X_k(1, :);
    Y_total(i+1, :) = Y_k(1, :);
    
    % pick the first CoP
    Z_x_total(i) = current_U(1);
    Z_y_total(i) = current_U(1+N);
    
    % update the initial state of CoM and CoP for next loop 
    x_hat_k = X_k(1, :);
    y_hat_k = Y_k(1, :);    
    Z_ref_k = planned_Z_ref(i+1:i+N, :);    
    T_k = T_k + delta_t;
end

%% plot results of the linear MPC
t = delta_t * (1:desired_walking_time);

min_admissible_cop = desired_Z_ref - [foot_length/2, foot_width/2] .* ones(desired_walking_time, 1);
max_admissible_cop = desired_Z_ref + [foot_length/2, foot_width/2] .* ones(desired_walking_time, 1);

% x direction
fig1 = figure(101); clf;
fig1.Position = [200 100 1000 750];
plot(t, Z_x_total, 'LineWidth', 3);
hold on
plot(t, X_total(1:end-1, 1), 'r-', 'LineWidth', 5);
plot(t, desired_Z_ref(:, 1), 'LineWidth', 3);
plot(t, min_admissible_cop(:, 1), 'r--', 'LineWidth', 2)
plot(t, max_admissible_cop(:, 1), 'm--', 'LineWidth', 2)
legend('CoP-Cal', 'CoM-Cal', 'CoP-Ref', 'min-CoP', 'max-CoP', 'Location', 'best')
xlabel('Time [ s ]');
ylabel('Displacement [ m ]');
grid on
set(gca, 'FontSize', 32);

% y direction
fig2 = figure(102); clf;
fig2.Position = [200 100 1000 750];
plot(t, Z_y_total, 'LineWidth', 3);
hold on
plot(t, Y_total(1:end-1, 1), 'r-', 'LineWidth', 5);
plot(t, desired_Z_ref(:, 2), 'LineWidth', 3);
plot(t, min_admissible_cop(:, 2), 'r--', 'LineWidth', 3)
plot(t, max_admissible_cop(:, 2), 'm--', 'LineWidth', 3)
legend('CoP-Cal', 'CoM-Cal', 'CoP-Ref', 'min-CoP', 'max-CoP', 'Location', 'best')
xlabel('Time [ s ]');
ylabel('Displacement [ m ]');
grid on
set(gca, 'FontSize', 32);

%% Animation
biped_walk_animation;