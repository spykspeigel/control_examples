clear all, close all, clc 

m=1;
M=5;
L=2;
g=-10;
d=10;

b = 1; % pendulum up (b=1)

A = [0 1 0 0;
    0 -d/M b*m*g/M 0;
    0 0 0 1;
    0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; b*1/(M*L)];

%p=[-1;-1.1;-1.2;-1.3];
p=[-2;-2.1;-2.2;-2.3];
K=place(A,B,p);
%Q=[1,0,0,0;0,1,0,0;0,0,10,0;0,0,0,100];
%R = .001;
%K=lqr(A,B,Q,R);

tspan = 0:.1:50;
y0 = [-3; 0; pi+0.1; 0];
[t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1;0;pi;0])),tspan, y0); 
f=figure(1);
%ylim([-2,2]);
for k=1:length(t)
	drawCartPend(y(k,:),k,L);
    clf(f);
end