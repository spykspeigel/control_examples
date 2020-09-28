clc; clear all; close all;


%addpath(genpath('/home/sagar/Documents/robotics/MPC-and-MHE-implementation-in-MATLAB-using-Casadi-master/workshop_github/Codes_casadi_v3_4_5/yalmip'));
addpath(genpath('path_to_casadi'))
import casadi.*
%%
p=1/2; %pick values like 1/4 loop, 1/2 loop, or 1-full loop and more
%%
% Settings for simulation time
DriftRadius = 20; % Radius of driftin meters
L=2*pi*DriftRadius; 
v=8; % speed of vehicle in m/s, this is usually applied in the kinematic bycicle model
vx=v; % longitudinal speed used in full car model.
t_loop=(L/v); %37.7 seconds is approximate time to do 1 revolution at v=5m/s
Tsim=t_loop*p; %Simulation time in seconds. the p factor gives us the number of loops.
N=10; % steps into horizon (in seconds it would be t-Horizon=N*dt)
T=0.1; %simulation time in seconds
%Physical parameters of Car
m=2300; % in Kg
Iz=4400; % from Johns paper.
a=1.5; lf=a; %length from CoG to front axle in meters
b=1.4; lr=b; %length from CoG to back axle in meters
h=0.2;

%%
%TIRE MODEL
Fz = (1/2)*(m*9.81)/1000; %force in K-newtons
a1 = -22.1; a2 = 1011; a3 = 1078; a4 = 1.82; a5 = 0.208; a6 = 0.000; a7 = -0.354; a8 = 0.707; C = 1.30;
D = a1*(Fz^2) + a2*Fz; BCD = a3*sin(a4*atan(a5*Fz)); B = BCD/(C*D); E = a6*(Fz^2) + a7*Fz + a8;

%%
%Declare State and Input Variables


x = SX.sym('x'); y = SX.sym('y'); psi = SX.sym('psi'); vy=SX.sym('vy');r=SX.sym('r');
states = [x;y;psi;vy;r]; n_states = length(states);

delta = SX.sym('delta');
controls = [delta]; n_controls = length(controls);

%%%%% Model definition
alphaF=atan((vy+ lf*r)/vx)-delta;        
alphaR = (atan((vy- lr*r)/vx));
FyF = D*sin(C*atan(B*alphaF));
FyR = D*sin(C*atan(B*alphaR));
rhs1=(vx*cos(psi)-vy*sin(psi));
rhs2=(vx*sin(psi)+vy*cos(psi));
rhs3=r;
rhs4=(FyF*cos(delta)+FyR)/m-vx*r;
rhs5=(lf*FyF*cos(delta)-lr*FyR)/Iz;

rhs=[rhs1;rhs2;rhs3;rhs4;rhs5];
f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + N*(n_states));
X = SX.sym('X',n_states,(N+1));



obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(5,5); Q(1,1) = .2;Q(2,2) = .2;Q(3,3) = 0;Q(4,4)=0;Q(5,5)=0; % weighing matrices (states)


st  = X(:,1); % initial state
g = [g;st-P(1:5)];% initial condition constraints

for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(5*k+1:5*k+5))'*Q*(st-P(5*k+1:5*k+5)); 
    st_next = X(:,k+1); 
    f_val = f(st,con);
    st_next_euler=st+T*f_val;
    g = [g;st_next-st_next_euler]; % compute constraints % new
end











OPT_variables = [reshape(X,5*(N+1),1);reshape(U,N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

%------------------------------------------------------
%Constraint Definition
%-----------------------------------------------------
args = struct;

% 
args.lbg(1:1:5*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:1:5*(N+1)) = 0;  % 1e-20   % Equality constraints
args.lbg(1:2:5*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:2:5*(N+1)) = 0;  % 1e-20   % Equality constraints
args.lbg(1:3:5*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3:5*(N+1)) = 0;  % 1e-20   % Equality constraints
args.lbg(1:4:5*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:4:5*(N+1)) = 0;  % 1e-20   % Equality constraints
args.lbg(1:5:5*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:5:5*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:5:5*(N+1),1) = -inf; %state x lower bound
args.ubx(1:5:5*(N+1),1) = inf; %state x upper bound
args.lbx(2:5:5*(N+1),1) = -inf; %state y lower bound
args.ubx(2:5:5*(N+1),1) = inf; %state y upper bound
args.lbx(3:5:5*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:5:5*(N+1),1) = inf; %state theta upper bound
args.lbx(4:5:5*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:5:5*(N+1),1) = inf; %state theta upper bound
args.lbx(5:5:5*(N+1),1) = -inf; %state theta lower bound
args.ubx(5:5:5*(N+1),1) = inf; %state theta upper bound

args.lbx(5*(N+1)+1:1:5*(N+1)+N,1) = -inf;%-40*pi/180; %v lower bound
args.ubx(5*(N+1)+1:1:5*(N+1)+N,1) = inf;%40*pi/180; %v upper bound


%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
x0 = [0 -DriftRadius 0 0 0]';    % initial condition.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,1);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables


sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];
x_res=[];
y_res=[];
main_loop = tic;
while(mpciter < Tsim / T) % new - condition for ending the loop
    current_time = mpciter*T;  %new - get the current time
    %----------------------------------------------------------------------
    args.p(1:5) = x0; % initial condition of the robot posture
    for k = 1:N %new - set the reference to track
        t_predict =  (k-1)*T; % predicted time instant
        %x_ref = DriftRadius*cos((vx/DriftRadius)*t_predict-pi/2); y_ref = DriftRadius*sin((vx/DriftRadius)*t_predict-pi/2);
        [x_ref,y_ref]=getref(x0(1),x0(2),DriftRadius,vx,t_predict);
        args.p(5*k+1:5*k+5) = [x_ref, y_ref, 0.0,0.0,0.0];

        %args.p(5*k+2:5*k+3) = [u_ref, omega_ref];
    end
    %----------------------------------------------------------------------    
    % initial value of the optimization variables
    args.x0  = [reshape(X0',5*(N+1),1);reshape(u0',N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(5*(N+1)+1:end))',1,N)'; % get controls only from the solution
    xx1(:,1:5,mpciter+1)= reshape(full(sol.x(1:5*(N+1)))',5,N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:5*(N+1)))',5,N+1)'; % get solution TRAJECTORY
    
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter
    mpciter = mpciter + 1;
end
main_loop_time = toc(main_loop);
average_mpc_time = main_loop_time/(mpciter+1);

Draw_MPC_tracking_v1 (t,xx,xx1,u_cl,N,0.3,DriftRadius)
