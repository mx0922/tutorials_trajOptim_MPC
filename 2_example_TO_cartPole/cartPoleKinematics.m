function [cart, pole] = cartPoleKinematics(z, p)

x = z(1, :);
q = z(2, :);

[p1x,p1y,p2x,p2y] = autoGen_cartPoleKinematics(x,q,p.l);

% p1y = 0.0 (scalar) --> vectorized
p1y = p1y * ones(1, length(p1x));

cart = [p1x; p1y];
pole = [p2x; p2y];

end