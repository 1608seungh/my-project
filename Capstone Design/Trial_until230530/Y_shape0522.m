% Raspi Robot Arm - Y shape

clear all; clc; clf;

PM = Func_0522temp;

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0;0;4], [0;4;4], [0;8;4], [0; 8; 2]];
M_end = [eye(3), [0; 8; 1]; 0 0 0 1];
th = [-63.5162; 40.5807; -64.1917; 23.6110; 63.5162]; % Start Pos : [6;3;2]
th = deg2rad(th);

PM.w = w; PM.q = q; PM.M_end = M_end;
pos = [[3;6], [1;6], [3;6], [3;6.5], [3;6], [7;2]; 2 2 2 2 2 2];

for k = 1:1:6
    x_d = pos(:,k);
    T_sd = [eye(3), x_d; 0 0 0 1];

    j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 4;

    A = 'start';
    disp(A)
    while (norm(w_b) > 0.01 || norm(v_b) > 0.001) && j < 150
        
        clf;
        PM.T_sd = T_sd; PM.th = th; PM = PM.calculate;
    
        v_b = PM.V_b(4:6);
        w_b = PM.V_b(1:3);
    
        gamma = -0.2;
%         th_home = [0; pi/6; pi/2; 0; 0];
        th_home = zeros(5,1);
        grad_H = zeros(5,1);
        th_home(3) = -pi/2;
        grad_H(3) = th(3) - th_home(3);
        dth = pinv(PM.Jb)*PM.V_b + gamma*(eye(5)-pinv(PM.Jb)*PM.Jb)*grad_H;

%         dth = pinv(PM.Jb)*PM.V_b + (eye(5)-pinv(PM.Jb)*PM.Jb) * gamma * (th_home-th);
        th = th + dth * dt;
        th_deg = rad2deg(th)'
        PM.T_sb
    
        X(j,k) = PM.Trs(1,4,end); 
        Y(j,k) = PM.Trs(2,4,end);
        Z(j,k) = PM.Trs(3,4,end);
    
        Xb = nonzeros(X(:));
        Yb = nonzeros(Y(:));
        Zb = nonzeros(Z(:));
    
        subplot(1,2,1)
        plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5);  
        hold on
        PM = PM.draw_xyz;
    
        subplot(1,2,2) 
        plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5);  
        view([0,0,1])
        PM.draw_xy;
        hold on
        PM.draw_frame(PM.T_sb);
    
        pause(0.001);
    
        j = j+1;
    
    end

end 