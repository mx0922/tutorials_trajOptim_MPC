% cart_pole_model dynamics derive

clear; close all; clc

syms M m g l 'real'           % model parameters
syms x q dx dq ddx ddq 'real' % states
syms u 'real'                 % control

% unit vector
%  j |
%    |_____ i
i = sym([1; 0]); 
j = sym([0; 1]);

e = sin(q) * i - cos(q) * j; % cart -> tip of pendulum

% state vector
% 以pendulum up为0参考线，顺时针旋转q为负，逆时针旋转q为正，动手画一下应该就很容易明白！
z   = [x; q];
dz  = [dx; dq];
ddz = [ddx; ddq];

% Kinematics
% position
p1 = x * i;
p2 = p1 + l * e;
% velocity
v1 = jacobian(p1, z) * dz;
v2 = jacobian(p2, z) * dz;

% Dynamics
% KE ：kinematic energy   PE ： potential energy
KE = 0.5 * M * dot(v1, v1) + 0.5 * m * dot(v2, v2);
PE = M * g * dot(p1, j) + m * g * dot(p2, j);

%% Lagrangian Dynamics Derivation
DK_Ddq = jacobian(KE, dz);
dDK_Ddq_dt = jacobian(DK_Ddq, z) * dz + jacobian(DK_Ddq, dz) * ddz;

DK_Dq = jacobian(KE, z);
DP_Dq = jacobian(PE, z);

eqns = dDK_Ddq_dt - DK_Dq' + DP_Dq';

% dynamics function: MM * [ddy; ddq] - FF = [u; 0]
% MM * ddq = FF
[MM, FF] = equationsToMatrix(eqns, ddz);
MM = simplify(MM);
FF = simplify(FF);

rhs = FF + [u; sym(0)];
dyn = simplify(MM\rhs);

%% matlab function output
matlabFunction(dyn(1), dyn(2),...
    'file', 'autoGen_cartPoleDynamics.m',...
    'var', {x, q, dx, dq, u, M, m, g, l}, ...
    'outputs', {'ddx', 'ddq'});

matlabFunction(p1(1), p1(2), p2(1), p2(2),...
    'file', 'autoGen_cartPoleKinematics.m',...
    'var', {x, q, l},...
    'outputs', {'p1x', 'p1y', 'p2x', 'p2y'});
