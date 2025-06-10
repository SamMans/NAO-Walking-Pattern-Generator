function [COM] = wpg_online(x0,sim_tim)
x0 = internal observer;
% THE CONTROLLER STARTS FROM HERE
xx(:,1) = x0; % xx contains the history of states
u0 = zeros(N,n_controls);  % two control inputs 

% Start MPC
xx1 = [];
u_cl=[];
args.p(1:9) = x0; % set the values of the parameters vector (initial state)
for k=1:N % set the values of the parameters vector (reference trajectory)
    count = k+1;
    if(k+1>size(ZMPx_input,1))
        count = size(ZMPx_input,1);
    end
    args.p(9+k) = ZMPx_input(count,1);
    args.p(9+k+N) = ZMPy_input(count,1);
end
args.x0 = reshape(u0',n_controls*N,1); % initial value of the optimization variables (controls)
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
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