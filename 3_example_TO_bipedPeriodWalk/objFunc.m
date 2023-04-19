function obj = objFunc(u,p)

nt = size(u, 2);

lambda = u(5:6, :);
fxy = zeros(2, nt);

% please see dynamics_ODE.m for explanation
for i = 1:nt
    fxy(:, i) = p.fc_span * lambda(:, i);
end

% compute the objective value
obj = autoGen_objFunc(u(1,:),u(2,:),u(3,:),u(4,:),fxy(1,:),fxy(2,:));

end