function ceq = cst_stepLength(xF,p)

qF = xF(1:7);

% final state
[~,P5_F] = autoGen_feetPos(...
    qF(1),qF(2),qF(3),qF(4),qF(5),qF(6),qF(7),...
    p.l1, p.l2, p.l3, p.l4, p.l5);

% 摆动脚在x轴方向的结束位置等于期望步长
% P5_F(1) = stepLength;
ceq = P5_F(1) - p.stepLength;

end