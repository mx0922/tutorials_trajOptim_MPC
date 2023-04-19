function ceq = cst_heelStrike(x0,xF,p)
% 施加考虑硬接触情况碰撞速度突变的周期步态约束

qF  = xF(1:7);
q0  = x0(1:7);
dqF = xF(8:14);
dq0 = x0(8:14);

% 求解碰撞时的MM矩阵和摆动脚即将与地面建立接触的接触点的Jacobian(仅与qF有关)
[MM, Jc] = autoGen_heelStrike(...
    qF(1),qF(2),qF(3),qF(4),qF(5),qF(6),qF(7),...
    p.m1, p.m2, p.m3, p.m4, p.m5, p.I1, p.I2, p.I3, p.I4, p.I5, ...
    p.l1, p.l2, p.l3, p.l4, p.l5, p.c1, p.c2, p.c3, p.c4, p.c5);

% Nc矩阵的求解请参考../reference/ETH_dynamics_lecture.pdf
Nc = eye(7) - MM^(-1) * Jc' * (Jc * MM^(-1) * Jc')^(-1) * Jc;

%%%%%%   周期步态约束--姿态和速度： %%%%%%
% 1.姿态约束：注意并没有把开始和结束姿态的x放进约束中
ceqPos = q0(2:7) - qF([2,3,6,7,4,5]);

% 2.速度约束： 这也是最重要的一个约束，硬接触(Hard Contact)的假设是：
% 碰撞发生前后机器人的状态q不发生突变，而状态的导数，也即是速度dq发生突变
% 用公式表示即是：dq(+) = Nc * dq(-) +:代表碰撞后, -:代表碰撞前
% 而这里dq(+)对应于优化中的dq0(上一周期的碰撞后)
%       dq(-)对应于优化中的dqF(下一周期的碰撞前)
% 故而约束表示为：dq0 = Nc * dqF
ceqVel = dq0 - Nc * dqF;

% pack up equality constraints
ceq = [ceqPos;ceqVel];  

end