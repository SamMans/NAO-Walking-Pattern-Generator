% --------------------------- Robot Simulation -------------------------%

% Construct the robot
NAO_Param; %---> Robot parameters initialization

b = Link(L(1)); RLJ0 = Link(L(2)); RLJ1 = Link(L(3)); RLJ2 = Link(L(4));
RLJ3 = Link(L(5)); RLJ4 = Link(L(6)); RLJ5 = Link(L(7));
LLJ0 = Link(L(8)); LLJ1 = Link(L(9)); LLJ2 = Link(L(10));
LLJ3 = Link(L(11)); LLJ4 = Link(L(12)); LLJ5 = Link(L(13)); %---> Links

Link_array = [b,RLJ0,RLJ1,RLJ2,RLJ3,RLJ4,RLJ5,LLJ0,LLJ1,LLJ2,LLJ3,LLJ4,LLJ5];

Robot = Humanoid(Link_array,FSR); %----> Full robot construction
Walk_config; %---> Robot controller configuration 

% Simulation through inverse dynamics (assuming no slipping & no double support)
global Rob_T; global Rob_th_d; global Rob_v; global Rob_th_dd; global Rob_a;
ZMP_plot = []; %--> ZMP history record
U_plot = []; %--> Joint torque history record
Stab_index = 0; %--> stability index (used to determine whether the robot is stable)
% Start simulation
for i = 1:size(Sim_ddOrient,2)
    Rob_T = TRANSLATE(Sim_Pel(i,:))*ROTATE(Sim_Orient(i),"y");
    Rob_th_d = [0,0,Sim_dOrient(i)]; Rob_th_dd = [0,0,Sim_ddOrient(i)];
    Rob_v = Sim_dPel(i,:); Rob_a = Sim_ddPel(i,:);
    Robot.Hum(2) = set_q(Robot.Hum(2),Sim_Q(i,1),Sim_dQ(i,1),Sim_ddQ(i,1)); 
    Robot.Hum(3) = set_q(Robot.Hum(3),Sim_Q(i,2),Sim_dQ(i,2),Sim_ddQ(i,2)); Robot.Hum(4) = set_q(Robot.Hum(4),Sim_Q(i,3),Sim_dQ(i,3),Sim_ddQ(i,3));
    Robot.Hum(5) = set_q(Robot.Hum(5),Sim_Q(i,4),Sim_dQ(i,4),Sim_ddQ(i,4)); Robot.Hum(6) = set_q(Robot.Hum(6),Sim_Q(i,5),Sim_dQ(i,5),Sim_ddQ(i,5)); 
    Robot.Hum(7) = set_q(Robot.Hum(7),Sim_Q(i,6),Sim_dQ(i,6),Sim_ddQ(i,6));
    Robot.Hum(8) = set_q(Robot.Hum(8),Sim_Q(i,7),Sim_dQ(i,7),Sim_ddQ(i,7)); 
    Robot.Hum(9) = set_q(Robot.Hum(9),Sim_Q(i,8),Sim_dQ(i,8),Sim_ddQ(i,8)); Robot.Hum(10) = set_q(Robot.Hum(10),Sim_Q(i,9),Sim_dQ(i,9),Sim_ddQ(i,9));
    Robot.Hum(11) = set_q(Robot.Hum(11),Sim_Q(i,10),Sim_dQ(i,10),Sim_ddQ(i,10)); Robot.Hum(12) = set_q(Robot.Hum(12),Sim_Q(i,11),Sim_dQ(i,11),Sim_ddQ(i,11)); 
    Robot.Hum(13) = set_q(Robot.Hum(13),Sim_Q(i,12),Sim_dQ(i,12),Sim_ddQ(i,12));
    [Robot,ZMP,stab] = Simulate(Robot,Sim_Sup(i));
    if(stab == false)
        % increment stability index in case ZMP appraoches support polygon boundaries
        Stab_index = Stab_index+1; 
    end
    ZMP_plot = [ZMP_plot; ZMP]; %---> Actual ZMP trajectory
    U_plot = [U_plot; [Robot.Hum(2).ug,Robot.Hum(3).ug,Robot.Hum(4).ug,Robot.Hum(5).ug,Robot.Hum(6).ug...
        ,Robot.Hum(7).ug,Robot.Hum(8).ug,Robot.Hum(9).ug,Robot.Hum(10).ug,Robot.Hum(11).ug,...
        Robot.Hum(12).ug,Robot.Hum(13).ug]]; %---> Joint torques during walking
end

if(Stab_index>200) 
    error("Gait is likely unstable"); %--> display gait errorif index exceeds a certain threshold
end


% Simulation test
%tic
%Robot = Simulate(Robot);
%toc

% Contact test
%T_pelvis = zeros(3,500);

%for i = 1:5/delta_t
%    [Robot, RES] = InvDyn(Robot,1);
%end

% IK Test
%global Rob_T;
%Rob_T = [1,0,0,1;
%    0,1,0,0;
%    0,0,1,0;
%    0,0,0,1];
%QS = InvKinl(Robot,Rob_T(1:3,4).',[1.02,0.035,0.03],0,pi/4);
%Robot.Hum(8) = set_q(Robot.Hum(8),QS(1),0,0); 
%Robot.Hum(9) = set_q(Robot.Hum(9),QS(2),0,0); Robot.Hum(10) = set_q(Robot.Hum(10),QS(3),0,0);
%Robot.Hum(11) = set_q(Robot.Hum(11),QS(4),0,0); Robot.Hum(12) = set_q(Robot.Hum(12),QS(5),0,0); 
%Robot.Hum(13) = set_q(Robot.Hum(13),QS(6),0,0);
%Robot = KinUpdate(Robot);
%T = Robot.Hum(13).Tg*[lft; 0; 0; 1];
%des = T(1:3,1).'

% Trial simulations (Chapter 3)
%Q = 0;
%Q_rasma = zeros(1,500);
%DQ = 0;
%DQ_rasma = zeros(1,500);
%DDQ = 0;
%DDQ_rasma = (1/5)*ones(1,500); 
%DDQ_rasma(1) = 0;
%delta = 0.01;
%rasma = [];

%for i = 1:500
%    Q_rasma(i) = Q;
%    DQ_rasma(i) = DQ;
%    Robot.Hum(4) = set_q(Robot.Hum(4),Q,DQ,DDQ);
%    Robot = InvDyn(Robot);
%    rasma = [rasma, Robot.Hum(4).ug];
%    Q = Q + DQ*delta;
%    DQ = DQ + DDQ*delta;
%    DDQ = 1/5;
%end

%Time = 0:0.01:4.99;




