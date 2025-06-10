function [COM] = wpg(T,z,x0,sim_tim,ZMP_input)

% CasADi v3.4.5
addpath('C:\Users\ZERONE\Desktop\Humanoid_Masters\Matlab libraries')
import casadi.*

g = 9.81; %gravitational constant
N = 100; % prediction horizon
C = [1 0 -z/g]; %Output Matrix


v_max = 1000; v_min = -v_max;

x = SX.sym('x'); x_dot = SX.sym('x_dot'); x_ddot = SX.sym('x_ddot');
states = [x;x_dot;x_ddot]; n_states = length(states);

v = SX.sym('v'); 
controls = [v]; n_controls = length(controls);
rhs = [x + x_dot*T + (x_ddot/2)*(T^2) + v*((T^3)/6);
    x_dot + (x_ddot*T) + v*((T^2)/2);
    x_ddot + v*T]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % linear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + N);
% parameters (which include the initial state and the reference ZMP trajectory)

X = SX.sym('X',n_states,(N+1));
% A Matrix that represents the states over the optimization problem.

% compute solution symbolically
X(:,1) = P(1:3); % initial state
for k = 1:N
    st = X(:,k);  con = U(:,k);
    st_next  = f(st,con);
    X(:,k+1) = st_next;
end
% this function to get the optimal trajectory knowing the optimal solution
ff=Function('ff',{U,P},{X});

obj = 0; % Objective function
g = [];  % constraints vector

Q = 5000000;  % weighing matrices (states)
R = 1; % weighing matrices (controls)
% compute objective
for k=1:N
    st = X(:,k);  con = U(:,k);
    if(k>1)
        Q = 100000 + abs(P(k+3) - P(k+2))*50000000;
    else
        Q = 100000 + abs(P(k+3) - P(1))*50000000;
    end
    obj = obj+(C*st-P(k+3))'*Q*(C*st-P(k+3)) + (con)'*R*(con); % calculate obj
end

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   %state x
end

% make the decision variables one column vector
OPT_variables = reshape(U,n_controls*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


args = struct;
% inequality constraints (state constraints)
args.lbg = -5;  % lower bound of the states x and y
args.ubg = 5;   % upper bound of the states x and y 

% input constraints
args.lbx(1:n_controls*N,1) = v_min; 
args.ubx(1:n_controls*N,1) = v_max; 


%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SETTING UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,n_controls);  % one control input 


% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];


% the main simulaton loop... it works as long as the error is greater
% than 10^-2 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while(mpciter < (sim_tim-T) / T)
    args.p(1:3)   = x0; % set the values of the parameters vector
    for k=1:N
        count = mpciter+k+1;
        if(mpciter+k+1>size(ZMP_input,1))
            count = size(ZMP_input,1);
        end
        args.p(3+k) = ZMP_input(count,1);
    end
    args.x0 = reshape(u0',n_controls*N,1); % initial value of the optimization variables
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',n_controls,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:3,mpciter+1)= full(ff_value)';
    
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;  
    mpciter
    mpciter = mpciter + 1;
end;
main_loop_time = toc(main_loop);
COM = xx(1,:);

average_mpc_time = main_loop_time/(mpciter+1)

t(mpciter+1) = t0;
end