clear; close all; clc

% 总时间
T = 1.0;

% 开始时刻(t=0): 位置、速度、加速度
p0 = 0;     v0 = 0;     a0 = 0;
% 中间时刻(t=T/2): 位置
pm = 0.2;
% 结束时刻(t=T): 位置、速度、加速度
p1 = 0.5;   v1 = 0;     a1 = 0;

% 六次多项式的系数
S = getPolyCoeff(T, p0, v0, a0, pm, p1, v1, a1);

% 细分时刻各点的位置、速度、加速度
dt = 0.05;
tSpan = 0:dt:T;

Nt = length(tSpan);
pos = zeros(Nt, 1);
vel = zeros(Nt, 1);
acc = zeros(Nt, 1);

for i = 1:Nt
    t = tSpan(i);
    % 得到对应时刻的位置、速度、加速度
    [pos(i), vel(i), acc(i)] = getSixOrderPoly(S, t);
end

% % 利用matlab的矩阵运算能力直接得到整段轨迹
% [pos, vel, acc] = getSixOrderPoly(S, tSpan');

figure(1001); clf;
subplot(3, 1, 1)
plot(tSpan, pos)
subplot(3, 1, 2)
plot(tSpan, vel)
subplot(3, 1, 3)
plot(tSpan, acc)