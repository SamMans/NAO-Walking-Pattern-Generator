function [COM] = wpg_upd(T,x0,sim_tim,ZMPx_input,ZMPy_input,zmin,zmax)

% CasADi v3.4.5
addpath('C:\Users\ZERONE\Desktop\Humanoid_Masters\Matlab libraries')
import casadi.*

G = 9.81; % gravitational constant
N = 100; % prediction horizon


v_max = 100; v_min = -v_max; %--> Jerk limits

% State space model states
x = SX.sym('x'); x_dot = SX.sym('x_dot'); x_ddot = SX.sym('x_ddot');
y = SX.sym('y'); y_dot = SX.sym('y_dot'); y_ddot = SX.sym('y_ddot');
z = SX.sym('z'); z_dot = SX.sym('z_dot'); z_ddot = SX.sym('z_ddot');
states = [x;x_dot;x_ddot;y;y_dot;y_ddot;z;z_dot;z_ddot]; n_states = length(states);

% Inputs to the system (jerks)
vx = SX.sym('vx'); vy = SX.sym('vy'); vz = SX.sym('vz'); 
controls = [vx;vy;vz]; n_controls = length(controls);

% Non linear model (relation between state t+1 and state t)
rhs = [x + x_dot*T + (x_ddot/2)*(T^2) + vx*((T^3)/6);
    x_dot + (x_ddot*T) + vx*((T^2)/2);
    x_ddot + vx*T;
    y + y_dot*T + (y_ddot/2)*(T^2) + vy*((T^3)/6);
    y_dot + (y_ddot*T) + vy*((T^2)/2);
    y_ddot + vy*T;
    z + z_dot*T + (z_ddot/2)*(T^2) + vz*((T^3)/6);
    z_dot + (z_ddot*T) + vz*((T^2)/2);
    z_ddot + vz*T]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % non-linear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + 2*N); % parameters (which include the initial state and the reference ZMP trajectory)

X = SX.sym('X',n_states,(N+1)); % A Matrix that represents the states over the optimization problem.

% compute solution symbolically
X(:,1) = P(1:9); % initial state
for k = 1:N 
    st = X(:,k);  con = U(:,k);
    st_next  = f(st,con);
    X(:,k+1) = st_next;
end
% This function is used to get the optimal trajectory knowing the optimal solution
ff=Function('ff',{U,P},{X});

obj = 0; % Objective function
g = [];  % constraints vector

Q = [5000000 0;
    0 5000000];  % weighing matrices (states)
R = [1 0 0;
    0 1 0;
    0 0 1]; % weighing matrices (controls)

% compute objective
for k=1:N
    st = X(:,k);  con = U(:,k);
    Pactx = st(1,1)-((st(3,1)*st(7,1))/(G+st(9,1))); %--> Actual ZMP (x-coordinate)
    Pacty = st(4,1)-((st(6,1)*st(7,1))/(G+st(9,1))); %--> Actual ZMP (y-coordinate)
    Z = [Pactx-P(k+9);Pacty-P(k+9+N)]; %--> ZMP error vector
    obj = obj + Z'*Q*Z + (con)'*R*(con); % calculate obj
end

% compute constraints
for k = 1:N+1   % constraints on COM height z
    g = [g ; X(7,k)];   %state z
end

% make the decision variables one column vector
OPT_variables = reshape(U,n_controls*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level = 0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


args = struct;
% inequality constraints (state constraints)
args.lbg = zmin;  % lower bound of the state z
args.ubg = zmax;   % upper bound of the state z

% input constraints
args.lbx(1:n_controls*N,1) = v_min; 
args.ubx(1:n_controls*N,1) = v_max; 


%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0; % Initial timing
xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,n_controls);  % two control inputs 

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... 
main_loop = tic;
while(mpciter < (sim_tim-T) / T)
    args.p(1:9) = x0; % set the values of the parameters vector (initial state)
    for k=1:N % set the values of the parameters vector (reference trajectory)
        count = mpciter+k+1;
        if(mpciter+k+1>size(ZMPx_input,1))
            count = size(ZMPx_input,1);
        end
        args.p(9+k) = ZMPx_input(count,1);
        args.p(9+k+N) = ZMPy_input(count,1);
    end
    args.x0 = reshape(u0',n_controls*N,1); % initial value of the optimization variables (controls)
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',n_controls,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:9,mpciter+1)= full(ff_value)'; % history of all trial states
    u_cl= [u_cl ; u(1,:)]; % array of controls (jerks)
    
    t(mpciter+1) = t0; 
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;  
    mpciter
    mpciter = mpciter + 1;
end
main_loop_time = toc(main_loop);
%COM = [xx(1,:);xx(4,:);xx(7,:)]; %--> final center of mass trajectory (positions only)
COM = xx; %--> final center of mass trajectory 
average_mpc_time = main_loop_time/(mpciter+1)

t(mpciter+1) = t0;
end