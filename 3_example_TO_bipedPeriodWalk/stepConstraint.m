function [c, ceq] = stepConstraint(x0,xF,p)

% Gait must be periodic
ceq1 = cst_heelStrike(x0,xF,p);

% Swing foot boundary velocity constraints
c = cst_swingFootVel(x0,xF,p);

% Step length constraint
ceq2 = cst_stepLength(xF,p); 

% Pack up equality constraints:
ceq = [ceq1;ceq2];

end