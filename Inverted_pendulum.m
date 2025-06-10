clear PID
% Simulation parameters
    sim_time = 15; % Simulation Time

% Parameters
    delta_t = 0.001; b = 0.0001; Jrot = 6 * 10^-8; Kt = 0.016; Kb = 0.031; L = 0.001;
    R = 1.846; NN = 49; 
    m = 5; g = 9.81; r = 0.2;

% Motor + Pendulum states
    I = zeros(1,(sim_time/delta_t)); %--> Current initially is zero
    Tor = zeros(1,(sim_time/delta_t)); %--> Torque initially is zero
    th = zeros(1,(sim_time/delta_t)); %--> angle is initially zero
    th(1) = (30/180)*pi;
    thdot = zeros(1,(sim_time/delta_t)); %--> angular velocity is initially zero
    thddot = zeros(1,(sim_time/delta_t)); %--> angular accelration is initially zero
    V = zeros(1,(sim_time/delta_t)); %--> Initial motor voltage

% DC motor state evolution
for i = 1:(sim_time/delta_t)
    u = 0.0*(0-th(i));
    V(i) = PID(u,th(i));
    Tor(i) = Kt * NN * I(i); % DC motor pure output torque
    Tor = Tor - b*thdot(i); % Net input torque (after damping)
    thddot(i) = (Tor(i)+m*g*r*sin(th(i)))/(m*r^2+Jrot); % Evolution of angular acceleration
    if(i~=(sim_time/delta_t))
        th(i+1) = th(i)+thdot(i)*delta_t; thdot(i+1) = thdot(i)+thddot(i)*delta_t; % Evolution of pendulum states
        I(i+1) = -((R*delta_t - L)/L) * I(i) - ((Kb*delta_t*NN)/L) * thdot(i) +...
            (delta_t/L) * V(i); % Evolution of motor current
    end
end



% Required functions 
function [U] = PID(setpoint,actual)
% Parameters
Fs = 1/0.1; % Sampling frequency 
limit_H = 24; % Motor Voltage Positive Limit
limit_L = -24; % Motor Voltage Negative Limit
Kp = 100; % Proportional Gain
Ki = 0; % Integral Gain
Kd = 0; % Derivative Gain

% Error
e = setpoint - actual;

persistent Ei; %past error 
if isempty(Ei)
Ei = 0;
end
 
persistent Si; %error summation roll
if isempty(Si)
 Si = 0;
end

% Position PID (output is Motor Voltage)
 Si = Si + Ki*e*(1/Fs);
 U = Kp*e + Si + (Kd*(e-Ei))/(1/Fs);
 Ei = e;
 
% Add PID limits (highest-lowest) 
 if(U>limit_H)
     U = limit_H;
 elseif(U<limit_L)
     U = limit_L;
 end
end