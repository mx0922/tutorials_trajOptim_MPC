function drawCartPole(~, z)

clf; hold on;

Cart_Width = 0.15;
Cart_Height = 0.05;

Pole_Width = 4;  %pixels

xBnd = [-0.1, 1.5];
yBnd = [-0.6, 0.6];

%%%% Plot Rails
plot([xBnd(1) xBnd(2)],-0.5*Cart_Height*[1,1],'k-','LineWidth',2)

cart = z(1:2);
pole = z(3:4);

%Plot Cart
x = cart(1) - 0.5*Cart_Width;
y = -0.5*Cart_Height;
w = Cart_Width;
h = Cart_Height;
hCart = rectangle('Position',[x,y,w,h],'LineWidth',2);
cartColor = [0 0 255] / 255;
set(hCart,'FaceColor',cartColor);
set(hCart,'EdgeColor',0.8*cartColor);

%Plot Pendulum
Rod_X = [cart(1), pole(1)];
Rod_Y = [cart(2), pole(2)];
poleColor = [255 0 255] / 255;
plot(Rod_X,Rod_Y,'k-','LineWidth',Pole_Width,'Color',poleColor)

%Plot Bob and hinge
plot(pole(1),pole(2),'k.','MarkerSize',40,'Color',poleColor)
plot(cart(1),cart(2),'k.','MarkerSize',10,'Color',0.5*cartColor)

axis([xBnd, yBnd]);
% axis('equal');
% axis manual;
axis off;

end