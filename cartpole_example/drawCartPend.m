function drawCartPend(y,time,L)
% drawCartPole(time,pos,extents)
%
% INPUTS:
%   time = [scalar] = current time in the simulation
%   pos = [4, 1] = [x1;y1;x2;y2]; = position of [cart; pole]
%   extents = [xLow, xUpp, yLow, yUpp] = boundary of the draw window
%
% OUTPUTS:
%   --> A pretty drawing of a cart-pole
%

%%%% Unpack the positions:
x1=y(1);
y1=0;
x2=x1+L*sin(y(3));
y2=-L*cos(y(3));
	


% Title and simulation time:
title(sprintf('t = %2.2f%',time));
hold on;
% Draw the rail that the cart-pole travels on
plot([-2,2],[0,0],'k-','LineWidth',2);

% Draw the cart:
plot(x1, y1, 'bs','MarkerSize',30,'LineWidth',5);

% Draw the pole:
plot([x1,x2], [y1, y2], 'r-','LineWidth',2);

% Draw the bob of the pendulum:
plot(x2, y2, 'ro','MarkerSize',22,'LineWidth',4);
ylim([-2,2]);
xlim([-5,5]);
% Format the axis so things look right:
%hold off
%axis equal; axis(extents); axis off;      %  <-- Order is important here
hold off;
drawnow


end	