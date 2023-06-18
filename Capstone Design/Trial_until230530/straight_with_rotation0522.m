% Raspi Robot Arm - Y shape

clear all; clc; clf;

PM = Func_0522temp;

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0;0;4], [0;4;4], [0;8;4], [0; 8; 2]];
M_end = [eye(3), [0; 8; 1]; 0 0 0 1];
th = [-63.5162; 40.5807; -64.1917; 23.6110; 63.5162]; % Start Pos : [6;3;2]

PM.w = w; PM.q = q; PM.M_end = M_end;

for t = 1:10
    ddt = 0.1;
    th = [th(1); th(2); th(3); th(4); th(5)-45*ddt]

    t = t+1;
    pause(0.001)
end
th = deg2rad(th);

x_d = [3; 6; 2];
T_sd = [PM.matrix_exp([0;0;1], -pi/4), x_d; 0 0 0 1];

j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 2;

while (norm(w_b) > 0.01 || norm(v_b) > 0.001) && j < 150
        
        clf;
        PM.T_sd = T_sd; PM.th = th; PM = PM.calculate;
    
        v_b = PM.V_b(4:6);
        w_b = PM.V_b(1:3);
    
        gamma = 0.2;
        th_home = [0; pi/6; pi/2; 0; 0];
        dth = pinv(PM.Jb)*PM.V_b + (eye(5)-pinv(PM.Jb)*PM.Jb) * gamma * (th_home-th);
        th = th + dth * dt;
        th_deg = rad2deg(th)'
        PM.T_sb
    
        X(j) = PM.Trs(1,4,end); 
        Y(j) = PM.Trs(2,4,end);
        Z(j) = PM.Trs(3,4,end);
    
        Xb = nonzeros(X(:));
        Yb = nonzeros(Y(:));
        Zb = nonzeros(Z(:));
    
        subplot(1,2,1)
        plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5);  
        hold on
        PM = PM.draw_xyz;
    
        subplot(1,2,2) 
        plot3(Xb,Yb,Zb, 'color', 'Black', 'Linewidth', 0.5);  
        view([0,0,1])
        PM.draw_xy;
    
        pause(0.001);
    
        j = j+1;
    
end

   