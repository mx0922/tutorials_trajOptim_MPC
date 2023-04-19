% MAIN.m  Example 3 --  Five-link biped walking trajectory optimization.
%
% This script sets up and then solves the optimal trajectory for the five
% link biped, assuming that the walking gait is compused of single-stance
% phases of motion connected by impulsive heel-strike (no double-stance or
% flight phases).
%
% The equations of motion and gradients are all derived by:
%   --> Derive_equations_biped_walk.m 
%
%
% =========================================================================
% Noted by Meng Xiang(孟祥): 
% M. Kelly models the biped robot as fixed-base by assuming that the
% stance foot is firmly attached to the ground, which is not accurate.
% 
% In this tutorial, we model the robot as floated-base by adding 2 floating
% base freedoms (i.e., x and y). The control inputs include not only the
% torque of actuated joints, but also the contact force at stance foot.
% 
% If you have any questions, you can refer to some robotics handbooks or
% send emails to bitmengxiang@gmail.com.
%
% CopyRights Reserved. 2023.
% =========================================================================

clear; close all; clc
% add path
addpath ../optimTraj_lite_lib

%% parameters
% 五连杆机器人参数 -- 结构体param
param = getPhysicalParameters();

% stepLength and stepTime
param.stepLength = 0.5; % meter
param.stepTime   = 0.4; % seconds

% span form of plannar friction cone
mass_total = param.m1 + param.m2 + param.m3 + param.m4 + param.m5;
param.fc_span = 1 / norm([param.miu, 1]) * mass_total * param.g * [param.miu, -param.miu; 1, 1];

%% problem.func
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% dynamics: dynamics equation of five-link plannar biped robot (ODE)
problem.func.dynamics =  @(t,x,u)( dynamics_ODE(t,x,u,param) );

% pathObj: path objective (there is no bndObj in this problem)
problem.func.pathObj  = @(t,x,u)( objFunc(u,param) );

% bndCst: boundary constraints 
problem.func.bndCst   = @(t0,x0,tF,xF)( stepConstraint(x0,xF,param) );

problem.func.pathCst  = @(t,x,u)( pathConstraint(x,param) );

%% problem.bounds
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = param.stepTime;
problem.bounds.finalTime.upp = param.stepTime;

% State
qLow = [-1; 0.5; -pi/2; -pi/2;  -pi; -pi/2;  -pi];
qUpp = [ 1; 1.5;  pi/2;  pi/2; -0.1;  pi/2; -0.1];
dqLow = -10*ones(7,1);
dqUpp =  10*ones(7,1);
problem.bounds.state.low = [qLow; dqLow];
problem.bounds.state.upp = [qUpp; dqUpp];
problem.bounds.initialstate.low = [qLow; dqLow];
problem.bounds.initialstate.upp = [qUpp; dqUpp];
problem.bounds.finalstate.low = [qLow; dqLow];
problem.bounds.finalstate.upp = [qUpp; dqUpp];

uMax = 100;  % Nm
problem.bounds.control.low = [-uMax*ones(4,1);   0;   0];
problem.bounds.control.upp = [ uMax*ones(4,1); inf; inf];

%% problem.guess
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% For now, just assume a linear trajectory between boundary values

% very intuitive initial guess

problem.guess.time = [0, param.stepTime];

q0 = [  0; 0.7518; deg2rad([0; 20; -40; 20; -40])];
qF = [0.5; 0.7518; deg2rad([0; 20; -40; 20; -40])];

dq0 = (qF-q0)/param.stepTime;
dqF = dq0;

problem.guess.state = [q0, qF; dq0, dqF];

problem.guess.control = zeros(6,2);  % Start with no control inputs

%% problem.options
%%%% optional method: trapezoid   hermiteSimpson

%%%%%%%%%%  optimization: round 2    %%%%%%%%%%%%%
problem.options(1).method = 'trapezoid'; % Select the transcription method
problem.options(1).trapezoid.nGrid = 20; % method-specific options

% problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
% problem.options(1).hermiteSimpson.nSegment = 15;

problem.options(1).nlpOpt.MaxIter = 50;

%%%%%%%%%%  optimization: round 2    %%%%%%%%%%%%%
% problem.options(2).method = 'trapezoid'; % Select the transcription method
% problem.options(2).trapezoid.nGrid = 30; % method-specific options

% problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
% problem.options(2).hermiteSimpson.nSegment = 30;

% problem.options(2).nlpOpt.MaxIter = 50;

%% Solve the problem
%%%%% THE KEY LINE:
soln = optimTraj(problem);

% Transcription Grid points:
t = soln(end).grid.time;
q = soln(end).grid.state(1:7,:);
dq = soln(end).grid.state(8:14,:);
u = soln(end).grid.control;

%% Animation
Anim.speed = 0.1;
Anim.plotFunc = @(t,q)( drawBipedRobot(q,param) );
Anim.verbose = true;
% Anim.video = true; % true or false(default)
animate(t,q,Anim);

%% Draw robot stop action
fig = figure(123); clf(fig);
set(0, 'DefaultFigureRenderer', 'opengl');
set(gcf, 'Color', 'white');

drawRobotStopAction(q,param);

%% Plot the optimized results
figure(2); clf;
subplot(1,3,1);
plot(t,q);
legend('x','y','q1','q2','q3','q4','q5');
xlabel('time')
ylabel('link angles')

subplot(1,3,2);
plot(t,u(1:4,:));
legend('u1','u2','u3','u4');
xlabel('time')
ylabel('joint torques')

subplot(1,3,3);
lambda = u(5:6, :);
nt = length(lambda);
fxy = zeros(2, nt);
for i = 1:nt
    fxy(:, i) = param.fc_span * lambda(:, i);
end
plot(t,fxy);
legend('fx', 'fy');
xlabel('time')
ylabel('contact force')

%% remove the path
rmpath ../optimTraj_lite_lib