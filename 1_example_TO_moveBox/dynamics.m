function dx = dynamics(x, u)
% [dx, dxGrad] = dynamics(x,u)
%
% Computes the dynamics (and gradients) for a 1d point mass on a
% friction-less plane with a force actuator.
%

% q = x(1,:);    % Position
dq = x(2, :);    % Velocity
ddq = u;         % Acceleration

dx = [dq; ddq];  % Pack up derivative of state

end