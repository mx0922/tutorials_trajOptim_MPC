function drawBox_plotBndState(q, p)

figure(123);
clf; hold on;

%%%% Plot Rails
plot([-1 3],[0 0],'k-','LineWidth',2)

axis equal; axis([-1,3,-1,1]); 
axis off;

q0 = q(1);
qF = q(2);

% Plot Box at x = 0 m
color = [0.2,0.7,0.1];
x = q0 - 0.5*p.length;
y = 0;
w = p.length;
h = p.height;
hBox = rectangle('Position',[x,y,w,h],'LineWidth',2);
set(hBox,'FaceColor',color);
set(hBox,'EdgeColor',0.8*color);

% Plot Box at x = 2 m
color = [0.2,0.7,0.1];
x = qF - 0.5*p.length;
y = 0;
w = p.length;
h = p.height;
hBox = rectangle('Position',[x,y,w,h],'LineWidth',2);
set(hBox,'FaceColor',color);
set(hBox,'EdgeColor',0.8*color);

text(-0.25, -0.2, 'x = 0 m', 'FontSize', 14);
text( 1.75, -0.2, 'x = 2 m', 'FontSize', 14);

drawnow;

end