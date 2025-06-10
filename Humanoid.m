classdef Humanoid
   properties(SetAccess = public)
      Hum  % Array of robot links
   end
   properties(SetAccess = private)
      Hfsr % FSR translations in foot frame
   end
   
   methods
      function obj = Humanoid(Par,fsr) %---> Constructor
         obj.Hum = [Par(1), Par(2), Par(3), Par(4), Par(5),...
             Par(6), Par(7), Par(8), Par(9), Par(10), Par(11),...
             Par(12), Par(13)]; 
         obj.Hfsr = fsr;
      end
      
      function obj = FWD(obj) %--> Update all link global transforms
          obj.Hum(1) = set_T(obj.Hum(1)); %--> set the local transform of base link (pelvis/ID=1)
          obj.Hum(1) = set_Tg(obj.Hum(1),obj.Hum(1).T); %-> set global transform (same as local one for base link)
          % iterate over children
          for i = 1:size(obj.Hum(1).child,2) 
              T = obj.Hum(1).T; %--> initial transform variable
              id = 1; %--> ID of base link
              c_id = obj.Hum(id).child(i); %--> child link ID
              while(c_id~=0) %--> continue until no children left
                  obj.Hum(c_id) = set_T(obj.Hum(c_id));
                  T = T * obj.Hum(c_id).T; %--> Update transform variable
                  obj.Hum(c_id) = set_Tg(obj.Hum(c_id),T); % save date in link
                  c_id = obj.Hum(c_id).child; %--> shift to next child
              end
          end
      end
      
      function QS = InvKinr(obj,pel,rf,orient,yawr) %--> compute inverse kinematics for right leg
          global z_spacing; global y_spacing; global lth;
          global ltb; global lft;
          T = TRANSLATE([-pel(1),-pel(2),-pel(3)])*ROTATE(-orient,"y");
          pel = T*[pel.';1];
          rf = T*[rf.';1];
          pel = pel(1:3,1).'; %--> Adjust frames to keep pelvis at the center 
          rf = rf(1:3,1).'; %--> transfer foot location and orientation to match new frame
          % Closed form solution, Geometric
          QS = zeros(1,6);
          if(yawr~=0)
              QS(1) = yawr;
              rf(2) = rf(2) + y_spacing;
              rf = ROTATE(-yawr,"y")*[rf.' ; 1]; % Transform the leg to the standard zero rad yaw
              rf = rf(1:3,1).';
              rf(2) = rf(2) - y_spacing;
          end
          QS(6) = -atan2((rf(2)-pel(2)+y_spacing),(pel(3)-z_spacing-(rf(3)+lft)));
          QS(2) = -QS(6);
          Z = (pel(3)-z_spacing-(rf(3)+lft))/cos(QS(2));
          X = rf(1) - pel(1);
          temp = acos((X^2 + Z^2 - lth^2 - ltb^2)/(2*lth*ltb));
          QS(4) = -temp;
          QS(5) = -(pi/2 - asin((lth*sin(temp))/sqrt(X^2+Z^2)) - atan2(Z,X));
          QS(3) = -QS(4)-QS(5);
      end
      
      function QS = InvKinl(obj,pel,lf,orient,yawl) %--> compute inverse kinematics for left leg
          global z_spacing; global y_spacing; global lth;
          global ltb; global lft;
          T = TRANSLATE([-pel(1),-pel(2),-pel(3)])*ROTATE(-orient,"y");
          pel = T*[pel.';1];
          lf = T*[lf.';1];
          pel = pel(1:3,1).'; %--> Adjust frames to keep pelvis at the center 
          lf = lf(1:3,1).'; %--> transfer foot location and orientation to match new frame
          % Closed form solution, Geometric
          QS = zeros(1,6);
          if(yawl~=0)
              QS(1) = yawl;
              lf(2) = lf(2) - y_spacing;
              lf = ROTATE(-yawl,"y")*[lf.' ; 1]; % Transform the leg to the standard zero rad yaw
              lf = lf(1:3,1).';
              lf(2) = lf(2) + y_spacing;
          end
          QS(6) = -atan2((lf(2)-pel(2)-y_spacing),(pel(3)-z_spacing-(lf(3)+lft)));
          QS(2) = -QS(6);
          Z = (pel(3)-z_spacing-(lf(3)+lft))/cos(QS(2));
          X = lf(1) - pel(1);
          temp = acos((X^2 + Z^2 - lth^2 - ltb^2)/(2*lth*ltb));
          QS(4) = -temp;
          QS(5) = -(pi/2 - asin((lth*sin(temp))/sqrt(X^2+Z^2)) - atan2(Z,X));
          QS(3) = -QS(4)-QS(5);
      end
      
      function obj = W_link(obj) %--> Update joint angular velocities w.r.t world frame
          global Rob_th_d;
          W = Rob_th_d; % Angular velocity of robot pelvis w.r.t world frame (body base)
          obj.Hum(1) = set_Wg(obj.Hum(1),W); % Angular velocity of base link is re-inserted from global variable
          % iterate over children
          for i = 1:size(obj.Hum(1).child,2) 
              W = obj.Hum(1).Wg; %--> initial angular velocity
              id = 1; %--> ID of base link
              c_id = obj.Hum(id).child(i); %--> child link ID
              while(c_id~=0) %--> continue until no children left
                  w = obj.Hum(c_id).Tg(1:3,1:3)*[0;0;1]; % unit vector of the child link joint
                  W = W + (w*obj.Hum(c_id).dq).'; % Compute global angular velocity by adding past to current
                  obj.Hum(c_id) = set_Wg(obj.Hum(c_id),W); % Update global angular velocity of link
                  c_id = obj.Hum(c_id).child; %--> shift to next child
              end
          end
      end
      
      function obj = Jacv(obj) %--> Update all link frame linear speeds w.r.t world frame
          global Rob_v;
          Vel = Rob_v; %--> Linear velocity of robot pelvis w.r.t world frame (body base)
          obj.Hum(1) = set_Vg(obj.Hum(1),Vel); % Linear velocity of base link is re-inserted from global variable
          % iterate over children
          for i = 1:size(obj.Hum(1).child,2) 
              Vel = obj.Hum(1).Vg; %--> initial linear velocity
              id = 1; %--> ID of base link
              c_id = obj.Hum(id).child(i); %--> child link ID
              while(c_id~=0) %--> continue until no children left
                  Vel = Vel + cross(obj.Hum(obj.Hum(c_id).parent).Wg,(obj.Hum(c_id).Tg(1:3,4)-obj.Hum(obj.Hum(c_id).parent).Tg(1:3,4)).');
                  obj.Hum(c_id) = set_Vg(obj.Hum(c_id),Vel); %--> Compute accumulative velocity and save it in corresponding link
                  c_id = obj.Hum(c_id).child; %--> shift to next child
              end
          end
      end   
      
      function obj = Vo_link(obj) %--> Update all link spatial velocities w.r.t world frame
          Vo = obj.Hum(1).Vg - cross(obj.Hum(1).Wg,obj.Hum(1).Tg(1:3,4).'); %--> Pelvis spatial velocity
          obj.Hum(1) = set_Vog(obj.Hum(1),Vo); % Spatial velocity of base link is re-inserted
          % iterate over children
          for i = 1:size(obj.Hum(1).child,2) 
              Vo = obj.Hum(1).Vog; %--> initial spatial velocity
              id = 1; %--> ID of base link
              c_id = obj.Hum(id).child(i); %--> child link ID
              while(c_id~=0) %--> continue until no children left
                  Vo = Vo + cross(obj.Hum(c_id).Tg(1:3,4).',obj.Hum(c_id).Wg);
                  obj.Hum(c_id) = set_Vog(obj.Hum(c_id),Vo); %--> Compute accumulative velocity and save it in corresponding link
                  c_id = obj.Hum(c_id).child; %--> shift to next child
              end
          end
      end
      
      function obj = dW_link(obj) %--> Update all joint angular accelerations w.r.t world frame
          global Rob_th_dd;
          dW = Rob_th_dd; %--> Angular acceleration of base/pelvis link
          obj.Hum(1) = set_dWg(obj.Hum(1),dW); % Angular acceleration of base link is re-inserted
          % iterate over children
          for i = 1:size(obj.Hum(1).child,2) 
              dW = obj.Hum(1).dWg; %--> initial angular acceleration
              id = 1; %--> ID of base link
              c_id = obj.Hum(id).child(i); %--> child link ID
              while(c_id~=0) %--> continue until no children left
                  w = (obj.Hum(c_id).Tg(1:3,1:3)*[0;0;1]).'; % unit vector of link joint
                  dw = cross(obj.Hum(obj.Hum(c_id).parent).Wg,w); % derivative of unit vector of parent
                  dW = dW + dw*obj.Hum(c_id).dq + w*obj.Hum(c_id).ddq; % Accumulative angular acceleration
                  obj.Hum(c_id) = set_dWg(obj.Hum(c_id),dW); %--> Compute accumulative acceleration and save it in corresponding link
                  c_id = obj.Hum(c_id).child; %--> shift to next child
              end
          end
      end
      
      function obj = dVo_link(obj) %--> Update all link spatial accelerations w.r.t world frame
          global Rob_a; %--> Pelvis acceleration w.r.t world frame
          mat1 = [0 -obj.Hum(1).dWg(1,3) obj.Hum(1).dWg(1,2);
                  obj.Hum(1).dWg(1,3) 0 -obj.Hum(1).dWg(1,1);
                  -obj.Hum(1).dWg(1,2) obj.Hum(1).dWg(1,1) 0]; % Intermediate variable
          mat2 = [0 -obj.Hum(1).Wg(1,3) obj.Hum(1).Wg(1,2);
                  obj.Hum(1).Wg(1,3) 0 -obj.Hum(1).Wg(1,1);
                  -obj.Hum(1).Wg(1,2) obj.Hum(1).Wg(1,1) 0]; % Intermediate variable
          dVo = Rob_a + obj.Hum(1).Tg(1:3,4).'*mat1 + obj.Hum(1).Vg*mat2; % Body spatial acceleration
          obj.Hum(1) = set_dVog(obj.Hum(1),dVo); % spatial acceleration of base link is re-inserted
          % iterate over children
          for i = 1:size(obj.Hum(1).child,2) 
              dVo = obj.Hum(1).dVog; %--> initial angular acceleration
              id = 1; %--> ID of base link
              c_id = obj.Hum(id).child(i); %--> child link ID
              while(c_id~=0) %--> continue until no children left
                  w = (obj.Hum(c_id).Tg(1:3,1:3)*[0;0;1]).'; % unit vector of link joint
                  sv = cross(obj.Hum(c_id).Tg(1:3,4).',w); % Intermediate variable
                  dsv = cross(obj.Hum(obj.Hum(c_id).parent).Wg,sv) + ...
                      cross(obj.Hum(obj.Hum(c_id).parent).Vog,w); % Derivative of intermediate variable
                  dVo = dVo + dsv*obj.Hum(c_id).dq + sv*obj.Hum(c_id).ddq; % Accumulative spatial acceleration
                  obj.Hum(c_id) = set_dVog(obj.Hum(c_id),dVo); %--> Compute accumulative acceleration and save it in corresponding link
                  c_id = obj.Hum(c_id).child; %--> shift to next child
              end
          end
      end
      
      function obj = COM_coord(obj) %----> Update COM coordinates of links
          for i = 1:13
              C = obj.Hum(i).Tg*[obj.Hum(i).COM;1]; %---> get COM coordinates in world frame through a homogeneous transformation
              C = [C(1,1), C(2,1), C(3,1)]; %----> Convert the COM location to vector form
              obj.Hum(i) = set_COMg(obj.Hum(i),C); %---> save in link data
          end
      end
      
      function obj = KinUpdate(obj) %--> Update the online kinematic states of the robot object
          obj = FWD(obj);
          obj = W_link(obj);
          obj = Jacv(obj);
          obj = Vo_link(obj);
          obj = dW_link(obj);
          obj = dVo_link(obj);
          obj = COM_coord(obj);
      end
      
      function C_rob = COM_track(obj) %----> get global position of robot COM
          obj = KinUpdate(obj);
          SumNum = 0; SumDen = 0; % get numerator and denominator of COM equation
          for i = 1:13
              % Iterate and get sums of all links
              SumNum = SumNum + obj.Hum(i).m * obj.Hum(i).COMg;
              SumDen = SumDen + obj.Hum(i).m;
          end
          C_rob = SumNum/SumDen;
      end
      
      function M_rob = M_calc(obj) %----> get robot mass
          M_rob = 0;
          for i = 1:13
              M_rob = M_rob + obj.Hum(i).m; % iterate and add robot link masses
          end
      end
      
      function obj = InvDyn(obj,Cr,Cl) %---> return the force and torque acting on parent through current link action
          
          %global Df; global Kf; global FSR_read; 
          global g;
          obj = KinUpdate(obj); %--> Update the online kinematic states of the robot object
          
          IDs = [7,13]; %--> IDs of extreme links (feet)
          P_body = [0;0;0;0;0;0]; %--> inverse dynamic force/moment acting on pelvis
          
          for i = 1:size(IDs,2) %--> iterate over the two extreme links 
              % Inertial and kinematic parameters
              c_cap = [0 -obj.Hum(IDs(i)).COMg(3) obj.Hum(IDs(i)).COMg(2);
                  obj.Hum(IDs(i)).COMg(3) 0 -obj.Hum(IDs(i)).COMg(1);
                  -obj.Hum(IDs(i)).COMg(2) obj.Hum(IDs(i)).COMg(1) 0]; % COM skew matrix
              I = obj.Hum(IDs(i)).Tg(1:3,1:3)*obj.Hum(IDs(i)).I*obj.Hum(IDs(i)).Tg(1:3,1:3).'; % Moment of inertia in global frame
              I_s = [obj.Hum(IDs(i)).m*eye(3), obj.Hum(IDs(i)).m*c_cap.';
                  obj.Hum(IDs(i)).m*c_cap, obj.Hum(IDs(i)).m*(c_cap*c_cap.')+I]; % Inertia matrix of link
              zeta_dot = [obj.Hum(IDs(i)).dVog.'; obj.Hum(IDs(i)).dWg.']; % Acceleration state
              zeta = [obj.Hum(IDs(i)).Vog.'; obj.Hum(IDs(i)).Wg.']; % Velocity state
              Wg_cap = [0 -obj.Hum(IDs(i)).Wg(3) obj.Hum(IDs(i)).Wg(2);
                  obj.Hum(IDs(i)).Wg(3) 0 -obj.Hum(IDs(i)).Wg(1);
                  -obj.Hum(IDs(i)).Wg(2) obj.Hum(IDs(i)).Wg(1) 0]; % Angular velocity skew matrix
              Vog_cap = [0 -obj.Hum(IDs(i)).Vog(3) obj.Hum(IDs(i)).Vog(2);
                  obj.Hum(IDs(i)).Vog(3) 0 -obj.Hum(IDs(i)).Vog(1);
                  -obj.Hum(IDs(i)).Vog(2) obj.Hum(IDs(i)).Vog(1) 0]; % Spatial velocity skew matrix
              zeta_cap = [Wg_cap zeros(3,3);Vog_cap Wg_cap]; % zeta skew matrix
              % Environmental forces (gravity and contact)
              Fe = [0;0;-obj.Hum(IDs(i)).m*g]; % Weight environmental force
              Te = cross(obj.Hum(IDs(i)).COMg,Fe.').'; % Weight environmental torque
              if(IDs(i)==7) %--> Right foot contact forces
                  Fe = Fe + Cr(1:3,:);
                  Te = Te + Cr(4:6,:);
              elseif(IDs(i)==13) %--> Left foot contact forces
                  Fe = Fe + Cl(1:3,:);
                  Te = Te + Cl(4:6,:);
              end
              P_effect = I_s*zeta_dot + zeta_cap*I_s*zeta - [Fe;Te]; % Compute force/moment acting on child (foot) from parent
              obj.Hum(IDs(i)) = set_fg(obj.Hum(IDs(i)),P_effect(1:3,:)); % set force value for link
              obj.Hum(IDs(i)) = set_tg(obj.Hum(IDs(i)),P_effect(4:6,:)); % set torque value for link
              w = (obj.Hum(IDs(i)).Tg(1:3,1:3)*[0;0;1]).'; % get the unit vector of joint in world frame
              P = obj.Hum(IDs(i)).Tg(1:3,4).'; % Translation of link frame relative to world frame
              sv = cross(P,w); % Intermediate variable
              u = sv*obj.Hum(IDs(i)).fg + w*obj.Hum(IDs(i)).tg; % calculate joint torque
              obj.Hum(IDs(i)) = set_ug(obj.Hum(IDs(i)),u); % set joint torque
              P_id = obj.Hum(IDs(i)).parent; % Parent ID
              while(P_id~=0) %--> continue until no parents left
                  % Inertial and kinematic parameters
                  c_cap = [0 -obj.Hum(P_id).COMg(3) obj.Hum(P_id).COMg(2);
                      obj.Hum(P_id).COMg(3) 0 -obj.Hum(P_id).COMg(1);
                      -obj.Hum(P_id).COMg(2) obj.Hum(P_id).COMg(1) 0]; % COM skew matrix
                  I = obj.Hum(P_id).Tg(1:3,1:3)*obj.Hum(P_id).I*obj.Hum(P_id).Tg(1:3,1:3).'; % Moment of inertia in global frame
                  I_s = [obj.Hum(P_id).m*eye(3), obj.Hum(P_id).m*c_cap.';
                      obj.Hum(P_id).m*c_cap, obj.Hum(P_id).m*(c_cap*c_cap.')+I]; % Inertia matrix of link
                  zeta_dot = [obj.Hum(P_id).dVog.'; obj.Hum(P_id).dWg.']; % Acceleration state
                  zeta = [obj.Hum(P_id).Vog.'; obj.Hum(P_id).Wg.']; % Velocity state
                  Wg_cap = [0 -obj.Hum(P_id).Wg(3) obj.Hum(P_id).Wg(2);
                      obj.Hum(P_id).Wg(3) 0 -obj.Hum(P_id).Wg(1);
                      -obj.Hum(P_id).Wg(2) obj.Hum(P_id).Wg(1) 0]; % Angular velocity skew matrix
                  Vog_cap = [0 -obj.Hum(P_id).Vog(3) obj.Hum(P_id).Vog(2);
                      obj.Hum(P_id).Vog(3) 0 -obj.Hum(P_id).Vog(1);
                      -obj.Hum(P_id).Vog(2) obj.Hum(P_id).Vog(1) 0]; % Spatial velocity skew matrix
                  zeta_cap = [Wg_cap zeros(3,3);Vog_cap Wg_cap]; % zeta skew matrix
                  % Environmental forces (gravity)
                  Fe = [0;0;-obj.Hum(P_id).m*g]; % Weight environmental force
                  Te = cross(obj.Hum(P_id).COMg,Fe.').'; % Weight environmental torque  
                  if(P_id==1) %--> if parent link is body/pelvis
                      P_body = P_body + P_effect;
                      P_effect = I_s*zeta_dot + zeta_cap*I_s*zeta - [Fe;Te] + P_body; % Compute force/moment acting on child
                      obj.Hum(P_id) = set_fg(obj.Hum(P_id),P_effect(1:3,:)); % set force value for link
                      obj.Hum(P_id) = set_tg(obj.Hum(P_id),P_effect(4:6,:)); % set torque value for link
                      obj.Hum(P_id) = set_ug(obj.Hum(P_id),[]); % set joint torque
                  else
                      P_effect = I_s*zeta_dot + zeta_cap*I_s*zeta - [Fe;Te] + P_effect; % Compute force/moment acting on child
                      obj.Hum(P_id) = set_fg(obj.Hum(P_id),P_effect(1:3,:)); % set force value for link
                      obj.Hum(P_id) = set_tg(obj.Hum(P_id),P_effect(4:6,:)); % set torque value for link
                      w = (obj.Hum(P_id).Tg(1:3,1:3)*[0;0;1]).'; % get the unit vector of joint in world frame
                      P = obj.Hum(P_id).Tg(1:3,4).'; % Translation of link frame relative to world frame
                      sv = cross(P,w); % Intermediate variable
                      u = sv*obj.Hum(P_id).fg + w*obj.Hum(P_id).tg; % calculate joint torque
                      obj.Hum(P_id) = set_ug(obj.Hum(P_id),u); % set joint torque
                  end
                  P_id = obj.Hum(P_id).parent; % shift to next parent
              end
          end
      end
      
      function [obj,Acc] = FwdDyn(obj) %--> compute forward dynamics of the robot to get the real motion pattern
          
          global Rob_th_d; global Rob_v; global Rob_th_dd; global Rob_a; global Rob_T;
          nDOF = size(obj.Hum,2)-1+6; %--> no. of degrees of freedom of robot
          % Compute robot inverse dynamics at time t
          obj = InvDyn(obj);
          Ug = zeros(nDOF,1);
          Ug(1:6,1) = [obj.Hum(1).fg; obj.Hum(1).tg]; %--> fill the vector of acting forces and moments
          for i=2:size(obj.Hum,2)
              Ug(5+i,1) = obj.Hum(i).ug;
          end
          
          % Compute equation of motion matrices
          A = zeros(nDOF,nDOF); %--> Inertia matrix
          % get b
          Rob_th_dd = [0,0,0]; %--> reset angular acceleration of pelvis
          mat2 = [0 -Rob_th_d(1,3) Rob_th_d(1,2);
                  Rob_th_d(1,3) 0 -Rob_th_d(1,1);
                  -Rob_th_d(1,2) Rob_th_d(1,1) 0]; % Intermediate variable
          Rob_a = -Rob_v*mat2; % reset Body spatial acceleration
          for i = 2:size(obj.Hum,2)
              obj.Hum(i) = set_q(obj.Hum(i),obj.Hum(i).q, obj.Hum(i).dq, 0); %--> reset joint accelerations
          end
          obj = InvDyn(obj);
          b = zeros(nDOF,1);
          b(1:6,1) = [obj.Hum(1).fg; obj.Hum(1).tg];
          for i=2:size(obj.Hum,2)
              b(5+i,1) = obj.Hum(i).ug;
          end
          % get A 
          delta = zeros(nDOF,1);
          delta(1,1) = 1; %--> delta vector of unit vector method
          for i=1:nDOF
              Rob_th_dd = delta(4:6,1).';
              mat1 = [0 -Rob_th_dd(1,3) Rob_th_dd(1,2);
                  Rob_th_dd(1,3) 0 -Rob_th_dd(1,1);
                  -Rob_th_dd(1,2) Rob_th_dd(1,1) 0]; % Intermediate variable
              mat2 = [0 -Rob_th_d(1,3) Rob_th_d(1,2);
                  Rob_th_d(1,3) 0 -Rob_th_d(1,1);
                  -Rob_th_d(1,2) Rob_th_d(1,1) 0]; % Intermediate variable
              Rob_a = delta(1:3,1).'-Rob_T(1:3,4).'*mat1-Rob_v*mat2; % reformulate Body spatial acceleration
              for k = 2:size(obj.Hum,2)
                  obj.Hum(k) = set_q(obj.Hum(k),obj.Hum(k).q, obj.Hum(k).dq, delta(5+k,1)); % fill joint accelerations
              end
              obj = InvDyn(obj); %--> compute inverse dynamics based on delta vector
              A_col = zeros(nDOF,1); %--> ith column of inertia matrix A
              A_col(1:6,1) = [obj.Hum(1).fg; obj.Hum(1).tg];
              for k=2:size(obj.Hum,2)
                  A_col(5+k,1) = obj.Hum(k).ug;
              end
              A(:,i) = A_col - b;
              delta(i,1) = 0;
              if(i<nDOF)
                 delta(i+1,1) = 1; %--> shift delta vector to the right
              end
          end
          Acc = inv(A)*(Ug-b); %--> compute actual accelerations of robot
      end    
      
      function [obj,ZMP,stability] = Simulate(obj,SUP) %--> Validate robot stability through simulation
          global g; %--> gravitational acceleration
          global lft; global us;
          obj = KinUpdate(obj); %--> Update Robot kinematics
          P_effect = [0;0;0;0;0;0]; %--> Forces and moments coming from parent side (initially zero)
          % Compute ZMP location and single support robot dynamics 
          if(SUP=="l")
              seq = [7,6,5,4,3,2,1,8,9,10,11,12,13]; %--> sequence of inverse dynamics links (ending with base link)
          else
              seq = [13,12,11,10,9,8,1,2,3,4,5,6,7];
          end
          for i = 1:size(seq,2) % Iterate over links starting with the branch, ending with the base
              c_cap = [0 -obj.Hum(seq(i)).COMg(3) obj.Hum(seq(i)).COMg(2);
                  obj.Hum(seq(i)).COMg(3) 0 -obj.Hum(seq(i)).COMg(1);
                  -obj.Hum(seq(i)).COMg(2) obj.Hum(seq(i)).COMg(1) 0]; % COM skew matrix
              I = obj.Hum(seq(i)).Tg(1:3,1:3)*obj.Hum(seq(i)).I*obj.Hum(seq(i)).Tg(1:3,1:3).'; % Moment of inertia in global frame
              I_s = [obj.Hum(seq(i)).m*eye(3), obj.Hum(seq(i)).m*c_cap.';
                  obj.Hum(seq(i)).m*c_cap, obj.Hum(seq(i)).m*(c_cap*c_cap.')+I]; % Inertia matrix of link
              zeta_dot = [obj.Hum(seq(i)).dVog.'; obj.Hum(seq(i)).dWg.']; % Acceleration state
              zeta = [obj.Hum(seq(i)).Vog.'; obj.Hum(seq(i)).Wg.']; % Velocity state
              Wg_cap = [0 -obj.Hum(seq(i)).Wg(3) obj.Hum(seq(i)).Wg(2);
                  obj.Hum(seq(i)).Wg(3) 0 -obj.Hum(seq(i)).Wg(1);
                  -obj.Hum(seq(i)).Wg(2) obj.Hum(seq(i)).Wg(1) 0]; % Angular velocity skew matrix
              Vog_cap = [0 -obj.Hum(seq(i)).Vog(3) obj.Hum(seq(i)).Vog(2);
                  obj.Hum(seq(i)).Vog(3) 0 -obj.Hum(seq(i)).Vog(1);
                  -obj.Hum(seq(i)).Vog(2) obj.Hum(seq(i)).Vog(1) 0]; % Spatial velocity skew matrix
              zeta_cap = [Wg_cap zeros(3,3);Vog_cap Wg_cap]; % zeta skew matrix
              Fe = [0;0;-obj.Hum(seq(i)).m*g]; % Weight environmental force
              Te = cross(obj.Hum(seq(i)).COMg,Fe.').'; % Weight environmental torque
              P_effect = I_s*zeta_dot + zeta_cap*I_s*zeta - [Fe;Te] + P_effect; % Compute force acting on child
              if(obj.Hum(seq(i)).ID~=1 && SUP~="d") % if link is not body/pelvis
                  obj.Hum(seq(i)) = set_fg(obj.Hum(seq(i)),P_effect(1:3,:)); % set force value for link
                  obj.Hum(seq(i)) = set_tg(obj.Hum(seq(i)),P_effect(4:6,:)); % set torque value for link
                  w = (obj.Hum(seq(i)).Tg(1:3,1:3)*[0;0;1]).'; % get the unit vector of joint in world frame
                  P = obj.Hum(seq(i)).Tg(1:3,4).'; % Translation of link frame relative to world frame
                  sv = cross(P,w); % Intermediate variable
                  u = sv*obj.Hum(seq(i)).fg + w*obj.Hum(seq(i)).tg; % calculate joint torque
                  obj.Hum(seq(i)) = set_ug(obj.Hum(seq(i)),u); % set joint torque
              elseif(obj.Hum(seq(i)).ID==1 && SUP~="d") % if link is pelvis
                  obj.Hum(seq(i)) = set_fg(obj.Hum(seq(i)),P_effect(1:3,:)); % set force value for link
                  obj.Hum(seq(i)) = set_tg(obj.Hum(seq(i)),P_effect(4:6,:)); % set torque value for link
                  obj.Hum(seq(i)) = set_ug(obj.Hum(seq(i)),[]); % set joint torque
              end
          end
          ZMPx = -P_effect(5,1)/P_effect(3,1); % X-coordinate of ZMP
          ZMPy = P_effect(4,1)/P_effect(3,1); % Y-coordinate of ZMP
          ZMP = [ZMPx,ZMPy]; % ZMP (global)
          % Compute robot dynamics (if double support)
          if(SUP=="d") 
              %--> Current distance between ZMP and right/left foot
              R_proj = obj.Hum(7).Tg*[lft;0;0;1]; % right foot sole center
              L_proj = obj.Hum(13).Tg*[lft;0;0;1]; % left foot sole center
              dfr = sqrt((ZMPx-R_proj(1,1))^2+(ZMPy-R_proj(2,1))^2); % distance between ZMP and right foot sole 
              dfl = sqrt((ZMPx-L_proj(1,1))^2+(ZMPy-L_proj(2,1))^2); % distance between ZMP and left foot sole
              %--> Observe ZMP trend & calculate force shares
              Cr = (dfl/(dfr+dfl))*P_effect; %--> share of right foot in contact force
              Cl = (dfr/(dfr+dfl))*P_effect; %--> share of left foot in contact force
              %--> Compute double support robot dynamics
              obj = InvDyn(obj,Cr,Cl);
          end
          % Check slipping (horizontal forces)
          if(abs(P_effect(1,1))>abs(us*P_effect(3,1))||abs(P_effect(2,1))>abs(us*P_effect(3,1)))
                  %error("Sliding");
          end
          Sup_poly = [obj.Hum(seq(size(seq,2))).Tg*[obj.Hfsr(1,:).';1],...
              obj.Hum(seq(size(seq,2))).Tg*[obj.Hfsr(2,:).';1],...
              obj.Hum(seq(size(seq,2))).Tg*[obj.Hfsr(3,:).';1],...
              obj.Hum(seq(size(seq,2))).Tg*[obj.Hfsr(4,:).';1]]; %--> Support polygon vertices
          Sup_poly = Sup_poly(1:2,:);
          xmin = min(Sup_poly(1,:));xmax= max(Sup_poly(1,:));ymin= min(Sup_poly(2,:));...
              ymax= max(Sup_poly(2,:)); %--> extreme values of vertices
          %--> Check whether zmp lies withing support polygon 
          % This check needs updates-->fix
          if(ZMPx>xmin && ZMPx<xmax && ZMPy>ymin && ZMPy<ymax)
              stability = true;
          else
              stability = false;
          end
      end
   end
end