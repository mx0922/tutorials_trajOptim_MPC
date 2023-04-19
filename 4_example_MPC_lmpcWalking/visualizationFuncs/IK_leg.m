function q = IK_leg(Body,D,A,B,Foot)
% Body和Foot的field要包含R和p，p:3*1, R:3*3
% D为AW/2, 左脚为+，右脚为-
% A为大腿长度，B为小腿长度
% 输出: 关节角度q - size:6*1

r = Foot.R' * (Body.p + Body.R * [0 D 0]'- Foot.p);  
C = norm(r);
c5 = (C^2-A^2-B^2)/(2.0*A*B);
if c5 >= 1 
    q5 = 0.0;
elseif c5 <= -1
    q5 = pi;
else
    q5 = acos(c5);  % knee pitch
end
q6a = asin((A/C)*sin(pi-q5));   % ankle pitch sub

q7 = atan2(r(2),r(3));  % ankle roll -pi/2 < q(6) < pi/2
if q7 > pi/2, q7=q7-pi; elseif q7 < -pi/2, q7=q7+pi; end
q6 = -atan2(r(1),sign(r(3))*sqrt(r(2)^2+r(3)^2)) -q6a; % ankle pitch

R = Body.R' * Foot.R * Rroll(-q7) * Rpitch(-q6-q5); %% hipZ*hipX*hipY
q2  = atan2(-R(1,2),R(2,2));   % hip yaw
cz = cos(q2); sz = sin(q2);
q3 = atan2(R(3,2),-R(1,2)*sz + R(2,2)*cz);  % hip roll
q4 = atan2( -R(3,1), R(3,3));               % hip pitch

q = [q2 q3 q4 q5 q6 q7]';

end
