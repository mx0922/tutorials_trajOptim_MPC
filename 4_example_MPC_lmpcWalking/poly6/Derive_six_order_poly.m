clear; close all; clc

syms a b c d e f g 'real'
syms p0 v0 p1 pm v1 a0 a1 'real'
syms t T 'real'

% 6次多项式：位置、速度、加速度
pos = a + b * t + c * t^2 + d * t^3 + e * t^4 + f * t^5 + g * t^6;
vel = diff(pos, t);
acc = diff(pos, t, 2);

% t=0: 位置、速度、加速度
t = 0;
eq_p0 = subs(pos) == p0;
eq_v0 = subs(vel) == v0;
eq_a0 = subs(acc) == a0;

% t=T/2: 位置
t = T/2;
eq_pm = subs(pos) == pm;

% t=T: 位置、速度、加速度
t = T;
eq_p1 = subs(pos) == p1;
eq_v1 = subs(vel) == v1;
eq_a1 = subs(acc) == a1;

% 求解多项式系数
S = solve([eq_p0, eq_v0, eq_a0, eq_pm, eq_p1, eq_v1, eq_a1], [a,b,c,d,e,f,g]);
      
a = S.a;
b = S.b;
c = S.c;
d = S.d;
e = S.e;
f = S.f;
g = S.g;

matlabFunction(...
    a, b, c, d, e, f, g, ...
    'file', 'autoGen_sixOrderPolyCoeff.m', ...
    'vars', {T, p0, v0, a0, pm, p1, v1, a1});