function [x,y]=getref(x_old,y_old,r,vx,t)
phi=atan2(y_old,x_old);
x=r*cos(vx/r*t+phi);
y=r*sin(vx/r*t+phi);

end