% Gait configuration
delta_t = 0.05; % gait sampling time
h_max = 0.01; % Maximum swing foot height above final landing position

% Support feet stamps for gait
% Robot starts motion from a global frame that lies in the middle of feet
Feet = [0 -0.05 0 0 1; %---> Start must have x=0 and orientation=0, other parameters are situation and workspace dependent
    0.05 0.05 0 0 2;   %---> Start must have abs(y-coordinate) = y_spacing --> standard beginning posture 
    0.1 -0.05 0 0 3;   %---> In this walking version, feet height must be zero always
    0.15 0.05 0 0 4;
    0.2 -0.05 0 0 5;
    0.25 0.05 0 0 6;
    0.3 -0.05 0 0 7;               %----> Excluding initial and final double support phases (Feet,Orient)
    0.35 0.05 0 0 8;
    0.4 -0.05 0 0 9;
    0.45 0.05 0 0 10;
    0.5 -0.05 0 0 11;
    0.55 0.05 0 0 12; 
    0.55 -0.05 0 0 13]; %--> Support feet info. (x,y,z,yaw orientation,timing in sec.)
Orient = [0,0,0,0,0,0,0,0,0,0,0,0,0]; %--> Pelvis orientation at each support event (start must be zero rad)

% ZMP trajectory
ZMP_traj = [];
for i = 1 : size(Feet,1)+1
    if(i==1) %--> switching from rest mode to walking mode
        for j = 0:delta_t:(Feet(i,5)-delta_t)
            ZMP_traj = [ZMP_traj; [Feet(i,1),0,Feet(i,3)]];
        end
    elseif(i>1 && i<=size(Feet,1)) %--> walking mode
        for j = Feet(i-1,5):delta_t:(Feet(i,5)-delta_t)
            ZMP_traj = [ZMP_traj; [Feet(i-1,1),Feet(i-1,2),Feet(i-1,3)]];
        end
    else %--> switching from walking mode to rest mode again
        for j = Feet(i-1,5):delta_t:(Feet(i-1,5)+Feet(1,5))-delta_t
            ZMP_traj = [ZMP_traj; [(Feet(i-1,1)+Feet(i-2,1))/2,(Feet(i-1,2)+Feet(i-2,2))/2,...
                Feet(i-1,3)]]; %--> Two feet must be evenly matched with the same orientation at final posture
        end
    end
end

% Feet trajectory
RF.traj = []; LF.traj = [];
if(Feet(1,2)<0) %--> Determine which foot is first support (right or left?)
    % Right foot
    RF.sup = 1; %--> Initialize right foot status to (support = true)
    LF.sup = 0; %--> Initialize left foot status to (support = false)
elseif(Feet(1,2)>0)
    RF.sup = 0; %--> Initialize right foot status to (support = false)
    LF.sup = 1; %--> Initialize left foot status to (support = true)
end
N = size(Feet,1); % Number of single support phases
Sup_map = [[Feet(1,1),-Feet(1,2),Feet(1,3),0,0];Feet;...
    [Feet(N-1,1:4),Feet(N,5)+Feet(1,5)]]; %---> list of all support feet including both single and double
for i = 1 : size(Sup_map,1)-1
    if(i==1)
        for j = Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t) %--> Transition from rest mode to walk mode
            RF.traj = [RF.traj; Sup_map(RF.sup+1,1:3)];
            LF.traj = [LF.traj; Sup_map(LF.sup+1,1:3)];
        end
    elseif(i>1 && i<=N) %--> Walk mode
        ti = Sup_map(i,5); %--> initial swing foot time
        tf = Sup_map(i+1,5); %--> final swing foot time
        tm = (ti+tf)/2; %--> mid-time for swing foot
        T = [ti^3,tf^3,3*ti^2,3*tf^2;
            ti^2,tf^2,2*ti,2*tf;
            ti,tf,1,1;
            1,1,0,0]; %--> Time matrix for x and y coordinates (cubic spline trajectory)
        M = [Sup_map(i-1,1),Sup_map(i+1,1),0,0;
            Sup_map(i-1,2),Sup_map(i+1,2),0,0]*inv(T); %--> Calculate cubic spline coefficients matrix (x & y)
        T = [ti^5,tm^5,tf^5,5*(ti^4),5*(tm^4),5*(tf^4);
            ti^4,tm^4,tf^4,4*(ti^3),4*(tm^3),4*(tf^3);
            ti^3,tm^3,tf^3,3*(ti^2),3*(tm^2),3*(tf^2);
            ti^2,tm^2,tf^2,2*ti,2*tm,2*tf;
            ti,tm,tf,1,1,1;
            1,1,1,0,0,0]; %--> Time matrix for z coordinates (power-5 spline trajectory)
        A = [Sup_map(i-1,3) Sup_map(i,3)+h_max Sup_map(i+1,3) 0 0 0]*inv(T); %--> power_5 spline coefficients matrix (z)
        for j = Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t)
            x_y = M*[j^3;j^2;j;1]; x = x_y(1,1); y = x_y(2,1); %--> x,y coordinates of swing foot
            z = A*[j^5;j^4;j^3;j^2;j;1]; %--> z coordinates of swing foot
            if(RF.sup) %--> if right foot is support
                RF.traj = [RF.traj; Sup_map(i,1:3)]; %--> support is fixed
                LF.traj = [LF.traj; [x,y,z]]; %---> swing is variable
            else %--> if left foot is support
                RF.traj = [RF.traj; [x,y,z]]; %--> support is fixed
                LF.traj = [LF.traj; Sup_map(i,1:3)]; %---> swing is variable
            end
        end
        RF.sup = 1-RF.sup; LF.sup = 1-LF.sup; %--> support exchange, flip sup value
    else
        for j = Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t)
            RF.traj = [RF.traj; Sup_map(i+1-RF.sup,1:3)]; %--> final double support 
            LF.traj = [LF.traj; Sup_map(i+1-LF.sup,1:3)]; %--> final double support
        end
    end
end

% Controller Configuration
% CasADi v3.4.5
addpath('C:\Users\ZERONE\Desktop\Humanoid_Masters\Matlab libraries')
import casadi.*

G = 9.81; % gravitational constant
N = 100; % prediction horizon
C_rob = COM_track(Robot); %----> get global position of robot COM in standard posture
zmin = 0.6*C_rob(3); zmax = 0.8*C_rob(3); %--> COM height bounds

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
rhs = [x + x_dot*delta_t + (x_ddot/2)*(delta_t^2) + vx*((delta_t^3)/6);
    x_dot + (x_ddot*delta_t) + vx*((delta_t^2)/2);
    x_ddot + vx*delta_t;
    y + y_dot*delta_t + (y_ddot/2)*(delta_t^2) + vy*((delta_t^3)/6);
    y_dot + (y_ddot*delta_t) + vy*((delta_t^2)/2);
    y_ddot + vy*delta_t;
    z + z_dot*delta_t + (z_ddot/2)*(delta_t^2) + vz*((delta_t^3)/6);
    z_dot + (z_ddot*delta_t) + vz*((delta_t^2)/2);
    z_ddot + vz*delta_t]; % system r.h.s

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
