% --------------------------- Walk Parameters -------------------------%
Robot = KinUpdate(Robot); %--> Update robot kinematic state based on standard standing posture defined in Param
C_rob = COM_track(Robot); %----> get global position of robot COM in standard posture
COMz_wpg = 0.7*C_rob(3); % COM height above the walking ground (for walking pattern generation purposes-emperical relation)
delta_t = 0.05; % gait sampling time
h_max = 0.01; % Maximum swing foot height above final landing position

% Support feet stamps for gait
% Robot starts motion from a global frame that lies in the middle of feet
Feet = [0 -0.05 0 0 1; %---> Start must have x=0 and orientation=0, other parameters are situation and workspace dependent
    0.05 0.05 0 0 2;   %---> Start must have abs(y-coordinate) = y_spacing --> standard beginning posture 
    0.1 -0.05 0 0 3;
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

% COM trajectory
COMx = wpg(delta_t,COMz_wpg,[0;0;0],Feet(size(Feet,1),5)+Feet(1,5),ZMP_traj(:,1));
COMy = wpg(delta_t,COMz_wpg,[0;0;0],Feet(size(Feet,1),5)+Feet(1,5),ZMP_traj(:,2));
COMz = [];
for i = 1 : size(Sup_map,1)-1
    if(Sup_map(i,1)==Sup_map(i+1,1))
        for j = Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t)
            COMz = [COMz, Sup_map(i,3)+COMz_wpg]; %--> both feet lying on the same level, support height doesn't change
        end
    else
        xi = Sup_map(i,1); %---> COM height follows a slope defined by start and end points (stair climbing)
        xf = Sup_map(i+1,1);
        X = linspace(xi,xf,(Sup_map(i+1,5)-Sup_map(i,5))/delta_t); %---> middle points used in substitution
        zi = Sup_map(i,3);   
        zf = Sup_map(i+1,3);  
        m = (zf-zi)/(xf-xi); %--> slope of COM trajectory in z
        k = 1; %--> counter
        for j = Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t)
            COMz = [COMz, m*(X(k)-xi) + zi + COMz_wpg]; %--> substitute in linear equation and get com height
            k = k+1;
        end
    end
end

% Hip yaw angles trajectories + Pelvis orientation trajectory
Orient_map = [0,Orient,Orient(size(Orient,2))];
Qr_yaw = []; Ql_yaw = []; %---> supports rotate to achieve required final swing foot landing orientation 
Pel_orient = []; %--> the pelvis might change its orientation w.r.t gait world frame
if(Feet(1,2)<0) %--> is right foot a support currently?
    R_sup = 1; %--> yes
else 
    R_sup = 0; %--> No
end
for i = 1 : size(Sup_map,1)-1
    if(i==1)
        for j = Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t) 
        %--> no change in rest to walk phase (both feet on the ground & pelvis with zero orientation)
            Qr_yaw = [Qr_yaw, 0];
            Ql_yaw = [Ql_yaw, 0];
            Pel_orient = [Pel_orient, 0];
        end
    else
        %--> delta change in pelvis orientation
        dorient = (Orient_map(i+1)-Orient_map(i))/size(Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t),2);
        %--> delta change in swing foot orientation
        dtheta = (Sup_map(i+1,4)-Sup_map(i-1,4))/size(Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t),2);
        
        Sump = Orient_map(i); %--> sum of pelvis angle increments added to initial orientation value
        Sumf = Sup_map(i-1,4); %--> sum of swing foot angle increments added to initial orientation value
        for j = Sup_map(i,5):delta_t:(Sup_map(i+1,5)-delta_t) %--> walk phase
            Sump = Sump + dorient;
            Sumf = Sumf + dtheta;
            Pel_orient = [Pel_orient, Sump];
            if(R_sup)
              Qr_yaw = [Qr_yaw, Sump-Sup_map(i,4)]; %--> add increments to right yaw
              Ql_yaw = [Ql_yaw, Sumf-Sump]; %--> add increments to left yaw
            else
              Ql_yaw = [Ql_yaw, Sump-Sup_map(i,4)]; %--> add increments to left yaw
              Qr_yaw = [Qr_yaw, Sumf-Sump]; %--> add increments to right yaw
            end
        end
        R_sup = 1-R_sup; %--> flip support for next exchange 
    end
end

Sim_Pel = []; %--> simulation pelvis trajectory of coordinates
% Compute joint angular trajectories (constraints on joint angles are missing!!!)
global Rob_T; global lth; global ltb; global lft, global z_spacing;
Q_traj = zeros(size(COMx,2),size(Robot.Hum,2)-1);
for i = 1:size(COMx,2) %--> iterate over computed trajectories so far to generate joint angles for each instant
    COM = [COMx(i),COMy(i),COMz(i)]; 
    Rob_T = TRANSLATE(COM)*ROTATE(Pel_orient(i),"y"); %--> Assume pelvis location matches COM location initially
    right = RF.traj(i,:); 
    left = LF.traj(i,:); 
    while(1) %--> Iterative optimization
        QS1 = InvKinr(Robot,Rob_T(1:3,4).',right,Pel_orient(i),Qr_yaw(i)); %--> right foot inverse kinematics
        Robot.Hum(2) = set_q(Robot.Hum(2),QS1(1),0,0); Robot.Hum(3) = set_q(Robot.Hum(3),QS1(2),0,0);
        Robot.Hum(4) = set_q(Robot.Hum(4),QS1(3),0,0); Robot.Hum(5) = set_q(Robot.Hum(5),QS1(4),0,0); 
        Robot.Hum(6) = set_q(Robot.Hum(6),QS1(5),0,0); Robot.Hum(7) = set_q(Robot.Hum(7),QS1(6),0,0);
        QS2 = InvKinl(Robot,Rob_T(1:3,4).',left,Pel_orient(i),Ql_yaw(i)); %--> left foot inverse kinematics
        Robot.Hum(8) = set_q(Robot.Hum(8),QS2(1),0,0); Robot.Hum(9) = set_q(Robot.Hum(9),QS2(2),0,0);
        Robot.Hum(10) = set_q(Robot.Hum(10),QS2(3),0,0); Robot.Hum(11) = set_q(Robot.Hum(11),QS2(4),0,0); 
        Robot.Hum(12) = set_q(Robot.Hum(12),QS2(5),0,0); Robot.Hum(13) = set_q(Robot.Hum(13),QS2(6),0,0);
        Robot = KinUpdate(Robot); %--> update robot kinematic states
        error = COM - COM_track(Robot); %--> compute error between desired and actual
        if(abs(error)>0.001)
            Rob_T(1:3,4) = Rob_T(1:3,4) + error.';
        else
            Q_traj(i,:) = [QS1,QS2];
            Sim_Pel = [Sim_Pel;Rob_T(1:3,4).'];
            break;
        end
    end
end
Rob_T = TRANSLATE([0,0,lth+ltb+lft+z_spacing])*ROTATE(0,"y"); %--> reset pelvis transform
% reset joint angles
Robot.Hum(2) = set_q(Robot.Hum(2),0,0,0); Robot.Hum(3) = set_q(Robot.Hum(3),0,0,0);
Robot.Hum(4) = set_q(Robot.Hum(4),0,0,0); Robot.Hum(5) = set_q(Robot.Hum(5),0,0,0); 
Robot.Hum(6) = set_q(Robot.Hum(6),0,0,0); Robot.Hum(7) = set_q(Robot.Hum(7),0,0,0);
Robot.Hum(8) = set_q(Robot.Hum(8),0,0,0); Robot.Hum(9) = set_q(Robot.Hum(9),0,0,0);
Robot.Hum(10) = set_q(Robot.Hum(10),0,0,0); Robot.Hum(11) = set_q(Robot.Hum(11),0,0,0); 
Robot.Hum(12) = set_q(Robot.Hum(12),0,0,0); Robot.Hum(13) = set_q(Robot.Hum(13),0,0,0);
Robot = KinUpdate(Robot); %--> Update robot kinematic state based on standard standing posture defined in Param

% Visualization environment inputs
Time_stamp = 0:delta_t:size(ZMP_traj,1)*delta_t-delta_t;
Time_stamp = Time_stamp.';
Q_2 = [Time_stamp,Q_traj(:,1)]; Q_3 = [Time_stamp,Q_traj(:,2)]; Q_4 = [Time_stamp,Q_traj(:,3)];
Q_5 = [Time_stamp,Q_traj(:,4)]; Q_6 = [Time_stamp,Q_traj(:,5)]; Q_7 = [Time_stamp,Q_traj(:,6)];
Q_8 = [Time_stamp,Q_traj(:,7)]; Q_9 = [Time_stamp,Q_traj(:,8)]; Q_10 = [Time_stamp,Q_traj(:,9)];
Q_11 = [Time_stamp,Q_traj(:,10)]; Q_12 = [Time_stamp,Q_traj(:,11)]; Q_13 = [Time_stamp,Q_traj(:,12)];
Hip_yaw = [Time_stamp,Pel_orient.']; Hip_x = [Time_stamp,Sim_Pel(:,1)]; Hip_y = [Time_stamp,Sim_Pel(:,2)];
Hip_z = [Time_stamp,Sim_Pel(:,3)];

% Generate required trajectory inputs for dynamic simulation
Sim_Orient = Pel_orient; %--> simulation pelvis trajectory of yaw angles orientation
Sim_dPel = diff(Sim_Pel)/delta_t; %--> simulation pelvis velocity
Sim_ddPel = diff(Sim_dPel)/delta_t; %--> simulation pelvis velocity
Sim_dOrient = diff(Sim_Orient)/delta_t; %--> simulation pelvis angle v
Sim_ddOrient = diff(Sim_dOrient)/delta_t; %--> simulation pelvis angle a
Sim_Q = Q_traj; % simulation angles
Sim_dQ = diff(Sim_Q)/delta_t; % simulation joint angular velocities
Sim_ddQ = diff(Sim_dQ)/delta_t; % simulation joint angular accelerations
Sim_Sup = [];
for i = 1 : (Feet(1,5))/delta_t
    Sim_Sup = [Sim_Sup, "d"];
end
for i = (Feet(1,5)+delta_t)/delta_t : Feet(size(Feet,1),5)/delta_t
    if(RF.traj(round(i),3)<=LF.traj(round(i),3))
        Sim_Sup = [Sim_Sup, "r"];
    else
        Sim_Sup = [Sim_Sup, "l"];
    end
end
for i =  (Feet(size(Feet,1),5)+delta_t)/delta_t : (Feet(size(Feet,1),5)+Feet(1,5))/delta_t
    Sim_Sup = [Sim_Sup, "d"];
end
% Save gaits in external excel files (if needed)
%xlswrite('Gait1_ddQ.xlsx',Sim_ddQ);