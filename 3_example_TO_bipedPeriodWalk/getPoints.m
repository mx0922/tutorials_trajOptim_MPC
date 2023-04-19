function [P, G] = getPoints(q,p)
% [P, G] = getPoints(q,p)
%
% This function computes the main points positions (P) and center of mass
% positions (G) for the five-link biped, in configuration given by q with
% physical parameters in p.
%
% INPUTS:
%   q = [7, n] = link configuration
%   p = parameters struct
%
% OUTPUTS:
%   P = [12, n] = points positions [x;y;x;y;...]
%   G = [10, n] = CoM positions {x;y;x;y;...]
%

x = q(1,:);  
y = q(2,:);
q1 = q(3,:); 
q2 = q(4,:);  
q3 = q(5,:);  
q4 = q(6,:);  
q5 = q(7,:);  

[P,G] = autoGen_kinematicsPoints(...
    x,y,q1,q2,q3,q4,q5,...
    p.l1 ,p.l2 ,p.l3 ,p.l4 ,p.l5 ,p.c1 ,p.c2 ,p.c3 ,p.c4 ,p.c5);

end