classdef Link
   properties(SetAccess = private)
   % Static properties
      name  % Link name
      ID % Link ID
      parent  % Parent link (link from which current link is derived)
      child % Child link (link derived fom current link)
      m  % Link mass
      COM  % Link COM relative to local link coordinates
      I  % Link moment of inertia (local frame)
      
   % Dynamic properties    
      q % Joint angle
      dq % Joint angular velocity
      ddq % Joint angular acceleration
      T % Link transform relative to child
      
   % Global properties (have to be updated and are dynamic)
      Tg; % Global-transform relative to world frame 
      Vg; % Global-frame linear velocity
      Wg; % Global-frame angular velocity
      dWg; % Global-frame angular acceleration
      Vog; % Global-frame spatial velocity
      dVog; % Global-frame spatial acceleration
      COMg; % Global- COM coordinates for link
      fg; % Global- summation of forces acting on link from parent
      tg; % Global- summation of moments about world frame origin from parent
      ug; % Joint torque
   end
   methods
      function obj = Link(Par) %---> Constructor
         obj.name = Par.name; obj.ID = Par.ID; obj.parent = Par.parent;   
         obj.child = Par.child; obj.m = Par.m; obj.COM = Par.COM; obj.I = Par.I;
         obj.q = Par.q; obj.dq = Par.dq; obj.ddq = Par.ddq;
         obj.T = Par.T; 
      end
      
      function obj = set_q(obj,x,y,z) %----> set the joint angles & update local transform
          obj.q = x;
          obj.dq = y;
          obj.ddq = z;
          set_T(obj);
      end
      
      function obj = set_T(obj) %----> set the transform of the link relative to parent
          global Rob_T; global y_spacing; global lth;
          global ltb; global z_spacing;
          switch obj.ID
              case 1
                  obj.T = Rob_T;
              case 2
                  obj.T = TRANSLATE([0,-y_spacing,-z_spacing])*ROTATE(obj.q,"y");
              case 3
                  obj.T = ROTATE(pi/2,"p")*ROTATE(obj.q,"y");
              case 4
                  obj.T = ROTATE(pi/2,"r")*ROTATE(obj.q,"y");
              case 5
                  obj.T = TRANSLATE([lth,0,0])*ROTATE(obj.q,"y");
              case 6
                  obj.T = TRANSLATE([ltb,0,0])*ROTATE(obj.q,"y");
              case 7
                  obj.T = ROTATE(-pi/2,"r")*ROTATE(obj.q,"y");
              case 8
                  obj.T = TRANSLATE([0,y_spacing,-z_spacing])*ROTATE(obj.q,"y");
              case 9
                  obj.T = ROTATE(pi/2,"p")*ROTATE(obj.q,"y");
              case 10
                  obj.T = ROTATE(pi/2,"r")*ROTATE(obj.q,"y");
              case 11
                  obj.T = TRANSLATE([lth,0,0])*ROTATE(obj.q,"y");
              case 12
                  obj.T = TRANSLATE([ltb,0,0])*ROTATE(obj.q,"y");
              case 13
                  obj.T = ROTATE(-pi/2,"r")*ROTATE(obj.q,"y");
          end
      end
      
      function obj = set_Tg(obj,x) %----> set the global transform of the link
          obj.Tg = x;
      end
      
      function obj = set_Vg(obj,x) %----> set the global velocity of the link
          obj.Vg = x;
      end
      
      function obj = set_Wg(obj,x) %----> set the global angular velocity of the link
          obj.Wg = x;
      end
      
      function obj = set_dWg(obj,x) %----> set the global angular acceleration of the link
          obj.dWg = x;
      end
      
      function obj = set_Vog(obj,x) %----> set the global spatial velocity of the link
          obj.Vog = x;
      end
      
      function obj = set_dVog(obj,x) %----> set the global spatial acceleration of the link
          obj.dVog = x;
      end
      
      function obj = set_COMg(obj,x) %----> set the global COM coordinates of the link
          obj.COMg = x;
      end  
      
      function obj = set_fg(obj,x) %----> set the global forces acting on the link from parent
          obj.fg = x;
      end 
      
      function obj = set_tg(obj,x) %----> set the global moments about global origin coming from the parent
          obj.tg = x;
      end 
      
      function obj = set_ug(obj,x) %----> set the magnitude of joint torque
          obj.ug = x;
      end  
   end
end