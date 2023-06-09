function [MM,Jac_c] = autoGen_heelStrike(x,y,q1,q2,q3,q4,q5,m1,m2,m3,m4,m5,I1,I2,I3,I4,I5,l1,l2,l3,l4,l5,c1,c2,c3,c4,c5)
%AUTOGEN_HEELSTRIKE
%    [MM,JAC_C] = AUTOGEN_HEELSTRIKE(X,Y,Q1,Q2,Q3,Q4,Q5,M1,M2,M3,M4,M5,I1,I2,I3,I4,I5,L1,L2,L3,L4,L5,C1,C2,C3,C4,C5)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    10-Mar-2023 20:48:31

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = cos(q5);
t7 = sin(q1);
t8 = sin(q2);
t9 = sin(q4);
t10 = q2+q3;
t11 = q4+q5;
t12 = c3.^2;
t13 = c5.^2;
t34 = m1+m2+m3+m4+m5;
t14 = cos(t10);
t15 = cos(t11);
t16 = sin(t10);
t17 = sin(t11);
t18 = c1.*m1.*t2;
t19 = c2.*m2.*t3;
t20 = c4.*m4.*t5;
t21 = c1.*m1.*t7;
t22 = c2.*m2.*t8;
t23 = c4.*m4.*t9;
t24 = m3.*t12;
t25 = m5.*t13;
t26 = l2.*t3.*2.0;
t27 = l4.*t5.*2.0;
t28 = l2.*t8.*2.0;
t29 = l4.*t9.*2.0;
t30 = c3.*l2.*m3.*t4;
t31 = c5.*l4.*m5.*t6;
t32 = l5.*t15;
t33 = l5.*t17;
t35 = -t18;
t36 = -t21;
t37 = c3.*t14.*2.0;
t38 = c5.*t15.*2.0;
t39 = c3.*t16.*2.0;
t40 = c5.*t17.*2.0;
t41 = c3.*m3.*t14;
t42 = c5.*m5.*t15;
t43 = c3.*m3.*t16;
t44 = c5.*m5.*t17;
t49 = I3+t24+t30;
t50 = I5+t25+t31;
t45 = t26+t37;
t46 = t27+t38;
t47 = t28+t39;
t48 = t29+t40;
t51 = (m3.*t45)./2.0;
t52 = (m5.*t46)./2.0;
t53 = (m3.*t47)./2.0;
t54 = (m5.*t48)./2.0;
t55 = t19+t51;
t56 = t20+t52;
t57 = t22+t53;
t58 = t23+t54;
MM = reshape([t34,0.0,t35,t55,t41,t56,t42,0.0,t34,t36,t57,t43,t58,t44,t35,t36,I1+c1.^2.*m1,0.0,0.0,0.0,0.0,t55,t57,0.0,I2+t30+t49+c2.^2.*m2+l2.^2.*m3,t49,0.0,0.0,t41,t43,0.0,t49,I3+t24,0.0,0.0,t56,t58,0.0,0.0,0.0,I4+t31+t50+c4.^2.*m4+l4.^2.*m5,t50,t42,t44,0.0,0.0,0.0,t50,I5+t25],[7,7]);
if nargout > 1
    Jac_c = reshape([1.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t32+l4.*t5,t33+l4.*t9,t32,t33],[2,7]);
end
