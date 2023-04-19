function drawBox(t, qu, p)

clf; hold on;

%%%% Plot Rails
plot([-1 3],[0 0],'k-','LineWidth',2)

axis equal; axis([-1,3,-1,1]); 
axis off;

q = qu(1);
u = qu(2);

%Plot Box
color = [0.2,0.7,0.1];
x = q - 0.5*p.length;
y = 0;
w = p.length;
h = p.height;
hBox = rectangle('Position',[x,y,w,h],'LineWidth',2);
set(hBox,'FaceColor',color);
set(hBox,'EdgeColor',0.8*color);

text(-0.25, -0.2, 'x = 0 m', 'FontSize', 14);
text( 1.75, -0.2, 'x = 2 m', 'FontSize', 14);

% plot the force(u)
quiver(q, 0.5*p.height, u, 0, 'r-', 'AutoScale', 'on', 'AutoScaleFactor', 1/15, 'LineWidth', 2.0);

title(sprintf('Move a Box Animation,  t = %6.3f', t));

drawnow; pause(0.001);

end