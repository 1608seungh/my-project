clear all; clc; 

PM = Func_0522temp;

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0; 0; 0.0701], [0; 0.1036; 0.0701], [0; 0.2004; 0.0701], [0; 0.2004; 0.0391]];
M_end = [eye(3), [0; 0.2004; 0.0068]; 0 0 0 1];

th = [-86.9474; 41.3064; -44.3338; 3.0274; 86.9474]; % Initial Angle
th = deg2rad(th);

j = 1; PM.w = w; PM.q = q; PM.M_end = M_end;
v_b = [1;1;1]; w_b = [1;1;1];

x_d = [0.15; 0.07; 0.0701]
T_sd = [eye(3), x_d; 0 0 0 1];

j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 2;

while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && j < 200
    
    PM.T_sd = T_sd;  PM.th = th; PM = PM.calculate;
        
    v_b = PM.V_b(4:6);
    w_b = PM.V_b(1:3);
    
    gamma = 0.2;
    th_home = [0; pi/3; 0; pi/3; 0];
    dth = pinv(PM.Jb)*PM.V_b + (eye(5)-pinv(PM.Jb)*PM.Jb) * gamma * (th_home-th);
    th = th + dth*dt;
    th_deg = rad2deg(th)' 
            
    j = j+1;

    PM.T_sb
end
