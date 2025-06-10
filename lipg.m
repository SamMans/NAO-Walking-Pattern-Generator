function [COM_traj, Feet]= lipg(Feet, zc, Tsup, com, comd, dt, Zp) 
    % Initial values
    n = 1; xi = com(1); yi = com(2); zmp = [Feet(1,1),Feet(1,2)];
    xdi = comd(1); ydi = comd(2); COM_traj = []; a = 100; b = 1;
    
    % Parameters
    g = 9.81; Tc = sqrt((zc+Zp)/g);
    C = cosh(Tsup/Tc);
    S = sinh(Tsup/Tc);
    D = a * (C - 1)^2 + b * (S / Tc)^2;
    
    while(n < size(Feet,1))
        % Calculate desired final walk parameters
        L = sqrt((Feet(n+1,1)-Feet(n,1))^2 + (Feet(n+1,2)-Feet(n,2))^2)/2;
        theta = atan2(Feet(n+1,2)-Feet(n,2),Feet(n+1,1)-Feet(n,1));
        x_d = L*cos(theta); y_d = L*sin(theta);
        xd_d = ((C+1)/(Tc*S))*x_d;
        yd_d = ((C-1)/(Tc*S))*y_d;
        x_d = x_d+zmp(1); y_d = y_d+zmp(2);

        % Calculate modified foot placement
        Feet(n,1) = - a * (C - 1) / D * (x_d - C * xi - Tc * S * xdi)...
             - b * S / (Tc * D) * (xd_d - S / Tc * xi - C * xdi);
        Feet(n,2) = - a * (C - 1) / D * (y_d - C * yi - Tc * S * ydi)...
             - b * S / (Tc * D) * (yd_d - S / Tc * yi - C * ydi);
        zmp(1) = Feet(n,1); zmp(2) = Feet(n,2);
            
        % Integrate LIPM equation
        for t = 0:dt:Tsup-dt
            x = (xi-zmp(1))*cosh(t/Tc) + Tc*xdi*sinh(t/Tc) + zmp(1);
            y = (yi-zmp(2))*cosh(t/Tc) + Tc*ydi*sinh(t/Tc) + zmp(2);
            xd = ((xi-zmp(1))/Tc)*sinh(t/Tc) + xdi*cosh(t/Tc);
            yd = ((yi-zmp(2))/Tc)*sinh(t/Tc) + ydi*cosh(t/Tc);
            COM_traj = [COM_traj, [x;y]];
        end
        
        % Update states
        xi_past = xi;
        yi_past = yi;
        xi = (xi-zmp(1))*C + Tc*xdi*S + zmp(1);
        yi = (yi-zmp(2))*C + Tc*ydi*S + zmp(2);
        xdi = ((xi_past-zmp(1))/Tc)*S + xdi*C;
        ydi = ((yi_past-zmp(2))/Tc)*S + ydi*C;
        n = n+1; zmp = [Feet(n,1),Feet(n,2)];        
    end
end