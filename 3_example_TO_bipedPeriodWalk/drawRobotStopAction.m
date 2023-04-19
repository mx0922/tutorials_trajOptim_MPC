function drawRobotStopAction(q,p)
% drawRobotStopAction(q,p)
%
% This function draws the robot with configuration q and parameters p over
% a series of configurations, leaving hold on so that the motion can be
% seen in a single (static) frame.
%
% INPUTS:
%   q = [7, 1] = column vector of a single robot configuration
%   p = parameter struct
%

nTime = size(q,2);

% Compute the points that will be used for plotting
[P, ~] = getPoints(q,p);

x = P(1:2:end,:);
y = P(2:2:end,:);

P0 = P(1:2,:);
P1 = P(3:4,:);
P2 = P(5:6,:);
% P3 = P(7:8,:);
P4 = P(9:10,:);
P5 = P(11:12,:);

% Heuristics:
L = (p.l2 + p.l3);  % Maximum extended leg length
xBnd = L*[-1.2,1.2];
yBnd = [-0.2*L, L + p.l1];

% Colors:
colorGround = [118,62,12]/255;
colorStance = [200,60,60]/255;
colorSwing = [60,60,200]/255;
colorTorso = [160, 80, 160]/255;

% Plot parameters:
legWidth = 2;
jointSize = 20;

% Set up the figure
hold off;

% Plot the ground:
plot(xBnd,[0,0],'LineWidth',6,'Color',colorGround);

hold on;

% Plot the links:
for i=1:nTime
plot(x([1,3,4],i),y([1,3,4],i),'--','LineWidth',legWidth,'Color',colorStance);
plot(0, 0,'k.','MarkerSize',jointSize,'Color',colorStance);
plot(P0(1,i), P0(2,i),'k.','MarkerSize',jointSize,'Color',colorStance);
plot(P2(1,i), P2(2,i),'k.','MarkerSize',jointSize,'Color',colorStance);
end
for i=1:nTime
plot(x(1:2,i),y(1:2,i),'LineWidth',legWidth+1,'Color',colorTorso);
plot(P1(1,i), P1(2,i),'k.','MarkerSize',jointSize,'Color',colorTorso);
end
for i=1:nTime
plot(x([1,5,6],i),y([1,5,6],i),'LineWidth',legWidth,'Color',colorSwing);
plot(P4(1,i), P4(2,i),'k.','MarkerSize',jointSize,'Color',colorSwing);
plot(P5(1,i), P5(2,i),'k.','MarkerSize',jointSize,'Color',colorSwing);
end

% Format the axis:
axis([xBnd,yBnd]); axis equal; axis off;

end