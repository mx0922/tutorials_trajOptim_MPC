function dx = dynamics_ODE(t,x,u,p)

nt = length(t);

% state
q   = x(1:7,:);        % position
dq  = x(8:14,:);       % velocity
ddq = zeros(size(q));  % acceleration

% control
tau    = u(1:4, :);    % joint torque
lambda = u(5:6, :);    % non-negative factor of friction cone span

for i = 1:nt
    % compute the contact force using fc_span and lambda
    fxy = p.fc_span * lambda(:, i);
    
    % get the matrix of dynamics equations
    [MM, rhs] = autoGen_dynamics(...
        q(1,i),q(2,i),q(3,i),q(4,i),q(5,i),q(6,i),q(7,i),...
        dq(1,i),dq(2,i),dq(3,i),dq(4,i),dq(5,i),dq(6,i),dq(7,i),...
        tau(1,i),tau(2,i),tau(3,i),tau(4,i),fxy(1),fxy(2),...
        p.m1, p.m2, p.m3, p.m4, p.m5, p.I1, p.I2, p.I3, p.I4, p.I5, ...
        p.l1, p.l2, p.l3, p.l4, p.l5, p.c1, p.c2, p.c3, p.c4, p.c5, p.g);
    
    % MM * ddz = rhs
    ddq(:, i) = MM \ rhs;    
end

dx = [dq; ddq];

end