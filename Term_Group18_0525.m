% 1st Term Project (Group 18)
clear all; clc; clf;

PM = Func_Group18;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0], [0;0;1]];
q = [[20;-20;0], [20;-20;10], [20;-50;10], [20;-70;10], [20; -80; 10], [20;-95;10]];
M_end = [eye(3), [20; -100; 10]; 0 0 0 1];

% Start Position -> pos(:, i) = [52; -25; 40]
% th_deg = [78.0074; -98.5113; 43.7745; 87.9881; 0; 11.6152];
th_deg = [79.2424; -97.8685; 42.3877; 88.3883; 0.1084; 10.7045];

th = deg2rad(th_deg);

PM.w = w; PM.q = q; PM.M_end = M_end;
pos = readmatrix('XYZ_Group18.xlsx', 'Range', 'B3:D103');
pos = transpose(pos);

for k = 1:length(pos)-1
    x_d = pos(:, k+1);
    T_sd = [eye(3), x_d; 0 0 0 1]; 
    
    j = 1; diff = 10; dt = 0.3; epsilon = 0.2;
    
    while diff > epsilon && j < 300  
    
        PM.T_sd = T_sd; PM.th = th; PM = PM.calculate;
    
        T_sb = PM.Trs(:,:,end);
        x = T_sb(1:3, 4);
        diff = norm(x_d-x);
    
        x_db = inv(T_sb) * [x_d; 1];
        x_db = x_db(1:3);
    
        v_b = x_db/norm(x_db)*0.5;
        Jbv = PM.Jb(4:6, :);
        dth = pinv(Jbv) * v_b;
        th = th + dth * dt;
    
        X(j,k) = PM.Trs(1,4,end); 
        Y(j,k) = PM.Trs(2,4,end);
        Z(j,k) = PM.Trs(3,4,end);

        Xb = nonzeros(X(:));
        Yb = nonzeros(Y(:));
        Zb = nonzeros(Z(:));

%         clf;
%         subplot(1,2,1)
%         PM = PM.draw_xyz;
%         plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5);  
%     
%         subplot(1,2,2) 
%         plot3(Xb,Yb,Zb, 'color', 'Black', 'Linewidth', 0.5);  
%         view([0,0,1])
%         PM.draw_xy;
%     
%         pause(0.001);
        j = j+1;
    end

end

        subplot(1,2,1)
        PM = PM.draw_xyz;
        plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5);  
    
        subplot(1,2,2) 
        plot3(Xb,Yb,Zb, 'color', 'Black', 'Linewidth', 0.5);  
        view([0,0,1])
        PM.draw_xy;