    %------------------------------------ NAO Parameters --------------------------------------%
                                              %clear;
% Robot dimensions
    global z_spacing; global y_spacing; global lth;
    global ltb; global lft;
    z_spacing = 0.085; % Spacing between pelvis center and legs in z-axis
    y_spacing = 0.05; % Spacing between pelvis center and legs in y-axis
    lth = 0.1; % Thigh length
    ltb = 0.1029; % Tibia length
    lft = 0.04519; % Foot height
    
% Robot Initial Pose, velocity and acceleration in world frame (Robot pose = Pelvis pose)
% The world frame is located on ground level beneath pelvis center (Height = 0 /or\ Datum)
    global Rob_T; global Rob_th_d; global Rob_v; global Rob_th_dd; global Rob_a;
    Rob_T = [1 0 0 0;
        0 1 0 0;
        0 0 1 lth+ltb+lft+z_spacing;
        0 0 0 1]; %---> Pelvis initial transform relative to world frame (standard posture)
    Rob_v = [0,0,0]; %---> Robot initial linear velocity w.r.t world frame
    Rob_th_d = [0,0,0]; %---> Robot initial angular velocity w.r.t world frame
    Rob_a = [0,0,0]; %---> Robot initial linear acceleration w.r.t world frame
    Rob_th_dd = [0,0,0]; %---> Robot initial angular acceleration w.r.t world frame
    
% Ground properties
    global us; global g;
    us = 0.9; % Coefficient of static friction
    g = 9.81; % Gravitational acceleration
    
% Robot sensors
% FSR translations from extreme links (limb ends)
    FSR = [[lft,0.07025,0.025];
        [lft,0.07025,-0.025];
        [lft,-0.03,0.025];
        [lft,-0.03,-0.025]];
    global FSR_read;
    FSR_read = [0,0,0,0];
    
% Simulation parameters
    global delta_t; delta_t = 0.01; % Simulation sampling time
    
% Link properties
% First link "Body"
    L(1).name = 'Body'; % Link name
    L(1).ID = 1; % Link ID
    L(1).parent = 0; % Parent link (link from which current link is derived)
    L(1).child = [2,8]; % Child link (link derived fom current link)
    L(1).q = []; % Link joint angle
    L(1).dq = []; % Link joint angular velocity
    L(1).ddq = []; % Link joint angular acceleration
    L(1).m = 2.98013; % Link mass
    L(1).COM = [0;0;0]; % Link COM relative to local link coord.
    L(1).I = [0.006 0 0;
              0 0.005 0;
              0 0 0.0021]; % Link moment of inertia (local frame)
    L(1).T = Rob_T; % Link transform with respect to parent (in this case, world frame)
          
% Second link "RLEG_J0"
    L(2).name = 'RLEG_J0'; % Link name
    L(2).ID = 2; % Link ID
    L(2).parent = 1; % Parent link (link from which current link is derived
    L(2).child = 3; % Child link (link derived fom current link)
    L(2).q = 0; % Link joint angle, Degree of freedom index
    L(2).dq = 0; % Link joint angular velocity
    L(2).ddq = 0; % Link joint angular acceleration
    L(2).m = 0.0000001; % Link mass
    L(2).COM = [0;0;0]; % Link COM relative to local link coord.
    L(2).I = [1*10^-6 0 0;
              0 1*10^-6 0;
              0 0 1*10^-6]; % Link moment of inertia (local frame)
    L(2).T = TRANSLATE([0,-y_spacing,-z_spacing])*ROTATE(L(2).q,"y"); % Link transform with respect to parent

% Third link "RLEG_J1"
    L(3).name = 'RLEG_J1'; % Link name
    L(3).ID = 3; % Link ID
    L(3).parent = 2; % Parent link (link from which current link is derived
    L(3).child = 4; % Child link (link derived fom current link)
    L(3).q = 0; % Link joint angle
    L(3).dq = 0; % Link joint angular velocity
    L(3).ddq = 0; % Link joint angular acceleration
    L(3).m = 0.1305; % Link mass
    L(3).COM = [0.00515;-0.00029;-0.01549]; % Link COM relative to local link coord.
    L(3).I = [8.81*10^-5 0 0;
              0 9.83*10^-5 0;
              0 0 2.758*10^-5]; % Link moment of inertia (local frame)
    L(3).T = ROTATE(pi/2,"p")*ROTATE(L(3).q,"y"); % Link transform with respect to parent 
          
% Fourth link "RLEG_J2"
    L(4).name = 'RLEG_J2'; % Link name
    L(4).ID = 4; % Link ID
    L(4).parent = 3; % Parent link (link from which current link is derived
    L(4).child = 5; % Child link (link derived fom current link)
    L(4).q = 0; % Link joint angle
    L(4).dq = 0; % Link joint angular velocity
    L(4).ddq = 0; % Link joint angular acceleration
    L(4).m = 0.38; % Link mass
    L(4).COM = [0.05373;0.00138;0.00221]; % Link COM relative to local link coord.
    L(4).I = [0.0003 0 0;
              0 0.0016 0;
              0 0 0.0016]; % Link moment of inertia (local frame)
    L(4).T = ROTATE(pi/2,"r")*ROTATE(L(4).q,"y"); % Link transform with respect to parent 
          
 % Fifth link "RLEG_J3"
    L(5).name = 'RLEG_J3'; % Link name
    L(5).ID = 5; % Link ID
    L(5).parent = 4; % Parent link (link from which current link is derived
    L(5).child = 6; % Child link (link derived fom current link)
    L(5).q = 0; % Link joint angle
    L(5).dq = 0; % Link joint angular velocity
    L(5).ddq = 0; % Link joint angular acceleration
    L(5).m = 0.29142; % Link mass
    L(5).COM = [0.04936;0.00453;0.00225]; % Link COM relative to local link coord.
    L(5).I = [0.00019 0 0;
              0 0.00118 0;
              0 0 0.001128]; % Link moment of inertia (local frame)
    L(5).T = TRANSLATE([lth,0,0])*ROTATE(L(5).q,"y"); % Link transform with respect to parent
          
 % Sixth link "RLEG_J4"
    L(6).name = 'RLEG_J4'; % Link name
    L(6).ID = 6; % Link ID
    L(6).parent = 5; % Parent link (link from which current link is derived
    L(6).child = 7; % Child link (link derived fom current link)
    L(6).q = 0; % Link joint angle
    L(6).dq = 0; % Link joint angular velocity
    L(6).ddq = 0; % Link joint angular acceleration
    L(6).m = 0.13146; % Link mass
    L(6).COM = [-0.00685;0.00045;-0.00685]; % Link COM relative to local link coord.
    L(6).I = [5*10^-5 0 0;
              0 3.85*10^-5 0;
              0 0 7.43*10^-5]; % Link moment of inertia (local frame)
    L(6).T = TRANSLATE([ltb,0,0])*ROTATE(L(6).q,"y"); % Link transform with respect to parent
          
 % Seventh link "RLEG_J5"
    L(7).name = 'RLEG_J5'; % Link name
    L(7).ID = 7; % Link ID
    L(7).parent = 6; % Parent link (link from which current link is derived
    L(7).child = 0; % Child link (link derived fom current link)
    L(7).q = 0; % Link joint angle
    L(7).dq = 0; % Link joint angular velocity
    L(7).ddq = 0; % Link joint angular acceleration
    L(7).m = 0.16184; % Link mass
    L(7).COM = [0.03239;-0.0033;0.02542]; % Link COM relative to local link coord.
    L(7).I = [0.000525 0 0;
              0 0.000643 0;
              0 0 0.000269]; % Link moment of inertia (local frame)
    L(7).T = ROTATE(-pi/2,"r")*ROTATE(L(7).q,"y"); % Link transform with respect to parent
          
 % Eightth link "LLEG_J0"
    L(8).name = 'LLEG_J0'; % Link name
    L(8).ID = 8; % Link ID
    L(8).parent = 1; % Parent link (link from which current link is derived
    L(8).child = 9; % Child link (link derived fom current link)
    L(8).q = 0; % Link joint angle
    L(8).dq = 0; % Link joint angular velocity
    L(8).ddq = 0; % Link joint angular acceleration
    L(8).m = 0.00000001; % Link mass
    L(8).COM = [0;0;0]; % Link COM relative to local link coord.
    L(8).I = [1*10^-6 0 0;
              0 1*10^-6 0;
              0 0 1*10^-6]; % Link moment of inertia (local frame)
    L(8).T = TRANSLATE([0,y_spacing,-z_spacing])*ROTATE(L(8).q,"y"); % Link transform with respect to parent   
          
 % Ninth link "LLEG_J1"
    L(9).name = 'LLEG_J1'; % Link name
    L(9).ID = 9; % Link ID
    L(9).parent = 8; % Parent link (link from which current link is derived
    L(9).child = 10; % Child link (link derived fom current link)
    L(9).q = 0; % Link joint angle
    L(9).dq = 0; % Link joint angular velocity
    L(9).ddq = 0; % Link joint angular acceleration
    L(9).m = 0.1305; % Link mass
    L(9).COM = [0.00515;0.00029;-0.01549]; % Link COM relative to local link coord.
    L(9).I = [8.81*10^-5 0 0;
              0 9.83*10^-5 0;
              0 0 2.758*10^-5]; % Link moment of inertia (local frame)
    L(9).T = ROTATE(pi/2,"p")*ROTATE(L(9).q,"y"); % Link transform with respect to parent
 
% Tenth link "LLEG_J2"
    L(10).name = 'LLEG_J2'; % Link name
    L(10).ID = 10; % Link ID
    L(10).parent = 9; % Parent link (link from which current link is derived
    L(10).child = 11; % Child link (link derived fom current link)
    L(10).q = 0; % Link joint angle
    L(10).dq = 0; % Link joint angular velocity
    L(10).ddq = 0; % Link joint angular acceleration
    L(10).m = 0.38968; % Link mass
    L(10).COM = [0.05373;0.00138;-0.00221]; % Link COM relative to local link coord.
    L(10).I = [0.0003 0 0;
              0 0.0016 0;
              0 0 0.0016]; % Link moment of inertia (local frame)
    L(10).T = ROTATE(pi/2,"r")*ROTATE(L(10).q,"y"); % Link transform with respect to parent   
          
 % Eleventh link "LLEG_J3"
    L(11).name = 'LLEG_J3'; % Link name
    L(11).ID = 11; % Link ID
    L(11).parent = 10; % Parent link (link from which current link is derived
    L(11).child = 12; % Child link (link derived fom current link)
    L(11).q = 0; % Link joint angle
    L(11).dq = 0; % Link joint angular velocity
    L(11).ddq = 0; % Link joint angular acceleration
    L(11).m = 0.29142; % Link mass
    L(11).COM = [0.04936;0.00453;-0.00225]; % Link COM relative to local link coord.
    L(11).I = [0.00019 0 0;
              0 0.00118 0;
              0 0 0.001128]; % Link moment of inertia (local frame)
    L(11).T = TRANSLATE([lth,0,0])*ROTATE(L(11).q,"y"); % Link transform with respect to parent
          
 % Twelveth link "LLEG_J4"
    L(12).name = 'LLEG_J4'; % Link name
    L(12).ID = 12; % Link ID
    L(12).parent = 11; % Parent link (link from which current link is derived
    L(12).child = 13; % Child link (link derived fom current link)
    L(12).q = 0; % Link joint angle
    L(12).dq = 0; % Link joint angular velocity
    L(12).ddq = 0; % Link joint angular acceleration
    L(12).m = 0.13146; % Link mass
    L(12).COM = [-0.00685;0.00045;0.00685]; % Link COM relative to local link coord.
    L(12).I = [5*10^-5 0 0;
              0 3.85*10^-5 0;
              0 0 7.43*10^-5]; % Link moment of inertia (local frame)
    L(12).T = TRANSLATE([ltb,0,0])*ROTATE(L(12).q,"y"); % Link transform with respect to parent
          
 % Thirteenth link "LLEG_J5"
    L(13).name = 'LLEG_J5'; % Link name
    L(13).ID = 13; % Link ID
    L(13).parent = 12; % Parent link (link from which current link is derived
    L(13).child = 0; % Child link (link derived fom current link)
    L(13).q = 0; % Link joint angle
    L(13).dq = 0; % Link joint angular velocity
    L(13).ddq = 0; % Link joint angular acceleration
    L(13).m = 0.16184; % Link mass
    L(13).COM = [0.03239;0.0033;0.02542]; % Link COM relative to local link coord.
    L(13).I = [0.000525 0 0;
              0 0.000643 0;
              0 0 0.000269]; % Link moment of inertia (local frame)
    L(13).T = ROTATE(-pi/2,"r")*ROTATE(L(13).q,"y"); % Link transform with respect to parent