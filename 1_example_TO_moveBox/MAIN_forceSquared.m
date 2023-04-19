% MAIN - Example: Move a box to the desired position in minimize force.
%
% Finds the optimal trajectory to slide a point-mass across a 1d
% frictionless plane.
%
% Simple force-squared cost function  --  This is easy to optimize
%

clear; close all; clc
addpath ../optimTraj_lite_lib

%% User-defined dynamics and objective functions
problem.func.dynamics = @(t,x,u)( dynamics(x,u) );       % ODE
problem.func.pathObj  = @(t,x,u)( obj_forceSquared(u) ); % objective function

% Problem bounds
% Move a box from x = 0 m to x = 2 m in 1 second.
% from stationary to stationary.

% bounds of initial and final time
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 1.0;
problem.bounds.finalTime.upp = 1.0;

% bounds of path state, initial state and final state
problem.bounds.state.low = [0; -inf];
problem.bounds.state.upp = [2;  inf];
problem.bounds.initialState.low = [0;0];
problem.bounds.initialState.upp = [0;0];
problem.bounds.finalState.low = [2;0]; 
problem.bounds.finalState.upp = [2;0];

% bounds of control
problem.bounds.control.low = -50; % force saturation
problem.bounds.control.upp =  50; % 

% Guess at the initial trajectory (initial guess)
problem.guess.time = [0, 1];
problem.guess.state = [0, 0; 2, 0];
problem.guess.control = [1, -1];

% Options for fmincon solver
problem.options.nlpOpt = optimset(...
    'Display','iter');   % show the iteration messenge

problem.options.method = 'trapezoid';    % trapeziod method
% problem.options.method = 'rungeKutta'; % rungeKutta method

%% Solve the problem
soln = optimTraj(problem);

t  = soln.grid.time;
q  = soln.grid.state(1,:);
dq = soln.grid.state(2,:);
u  = soln.grid.control;

%% Plot the solution:
figure(1); clf;

subplot(3,1,1)
plot(t,q)
ylabel('pos')
title('Move a Box');

subplot(3,1,2)
plot(t,dq)
ylabel('vel')

subplot(3,1,3)
plot(t,u)
ylabel('force')

%% Animate the solution
tGrid = soln.grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln.interp.state(t);
q = z(1, :);
u = soln.interp.control(t);

box.length = 0.4;
box.height = 0.2;

% Animate the results:
qu = [q; u]; % plot the force profile
A.plotFunc = @(t,qu)( drawBox(t,qu,box) );
A.speed = 0.25;
A.figNum = 101;
% A.video = true; % true or false(default)
animate(t,qu,A);

%% 
rmpath ../optimTraj_lite_lib