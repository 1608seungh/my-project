clear all; clc; 

PM = Func_0522temp;

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0; 0; 0.073], [0; 0.083; 0.073], [0; 0.166; 0.073], [0; 0.166; 0.033]];
M_end = [eye(3), [0; 0.166; -0.01]; 0 0 0 1];

th = [-86.9474; 41.3064; -44.3338; 3.0274; 86.9474]; % Initial Angle
th = deg2rad(th);

j = 1; PM.w = w; PM.q = q; PM.M_end = M_end;
v_b = [1;1;1]; w_b = [1;1;1];
pos = [[0.15; 0.07; 0.07], [0.02; 0.12; 0.07], [0.13; 0.09; 0.0701], [0.16; 0.09; 0.0701], [0.13; 0.09; 0.0701], [0.19; 0.03; 0.0701]];

for k = 1:1:3
    x_d = pos(:,k+1);
    T_sd = [eye(3), x_d; 0 0 0 1];

    j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 1;

    while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && j < 200
    
        PM.T_sd = T_sd;  PM.th = th; PM = PM.calculate;
    
        v_b = PM.V_b(4:6);
        w_b = PM.V_b(1:3);

        gamma = 0.2;

        th_center = zeros(5,1);
        th_center(3) = -pi/6;
        th_center(4) = pi/6;
        grad_H = zeros(5,1);
        grad_H(3) = th(3) - th_center(3);
        grad_H(4) = th(4) - th_center(4);

        Jvb = pinv(PM.Jb)*PM.V_b;
        dth = Jvb + gamma*(eye(5)-pinv(PM.Jb)*PM.Jb) * grad_H;
        th = th + dth*dt;
        th_deg = rad2deg(th)' 
        
        j = j+1;

    end
end