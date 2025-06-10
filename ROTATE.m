function M = ROTATE(x,y) %---> Rotation matrix function
if(y == "r") %---> roll rotation option
    M = [1 0 0 0;
        0 cos(x) -sin(x) 0;
        0 sin(x) cos(x) 0;
        0 0 0 1];
elseif(y == "p") %---> pitch rotation option
    M = [cos(x) 0 sin(x) 0;
        0 1 0 0;
        -sin(x) 0 cos(x) 0;
        0 0 0 1];
elseif(y == "y") %---> yaw rotation option
    M = [cos(x) -sin(x) 0 0;
        sin(x) cos(x) 0 0;
        0 0 1 0;
        0 0 0 1];
end
end