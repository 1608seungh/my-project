% Robot Kinematic 1st Term Project

clear all; clc; clf;

PM = Term_Project_Classdef;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0], [0;0;1]];
q = [[20;20;0], [20;20;10], [20;50;10], [20;70;10], [20; 80; 10], [20;95;10]];
M_end = [eye(3), [20; 100; 10]; 0 0 0 1];

% Start Position -> pos(:, i) = [52; 25; 40]
th_deg = [-75.9190; 123.0017; -104.0324; -22.3142; -0.7358; -31.0734];
th = deg2rad(th_deg);

PM.w = w; PM.q = q; PM.M_end = M_end;
pos = readmatrix('coordinates.xlsx', 'Range', 'B3:D24');
pos = transpose(pos);


for k = 1:1:21
    x_d = pos(:, k+1);
    T_sd = [eye(3), x_d; 0 0 0 1]; 
    
    j = 1; diff = 10; dt = 0.4; epsilon = 0.4;
    
    while diff > epsilon && j < 100  
    
        PM.T_sd = T_sd; PM.th = th; PM = PM.calculate;
    
        T_sb = PM.Trs(:,:,end);
        x = T_sb(1:3, 4);
        diff = norm(x_d-x);
    
        x_db = inv(T_sb) * [x_d; 1];
        x_db = x_db(1:3);
    
        v_b = x_db/norm(x_db)*2.5;
        Jbv = PM.Jb(4:6, :);
        dth = pinv(Jbv) * v_b;
        th = th + dth * dt;
        th_deg = rad2deg(th)';
    
        X(j,k) = PM.Trs(1,4,end); 
        Y(j,k) = PM.Trs(2,4,end);
        Z(j,k) = PM.Trs(3,4,end);

        XX = X(:);
        YY = Y(:);
        ZZ = Z(:);

        XX = nonzeros(XX);
        YY = nonzeros(YY);
        ZZ = nonzeros(ZZ);

        clf;
%         subplot(1,2,1)
%         PM = PM.draw_xyz;
%         plot3(XX,YY,ZZ, 'color', 'red', 'Linewidth', 0.5);  
%     
%         subplot(1,2,2) 
%         plot3(XX,YY,ZZ, 'color', 'Black', 'Linewidth', 0.5);  
%         view([0,0,1])
%         PM.draw_xy;
    
%         pause(0.001);
        j = j+1;
    end

end

        subplot(1,2,1)
        PM = PM.draw_xyz;
        plot3(XX,YY,ZZ, 'color', 'red', 'Linewidth', 0.5);  
    
        subplot(1,2,2) 
        plot3(XX,YY,ZZ, 'color', 'Black', 'Linewidth', 0.5);  
        view([0,0,1])
        PM.draw_xy;