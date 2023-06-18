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

% 1st Path 
x_d = pos(:, 2);
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; diff = 10; dt = 0.4; epsilon = 0.4;

while diff > epsilon && k < 100  

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

    x0(k) = PM.Trs(1,4,end); y0(k) = PM.Trs(2,4,end); z0(k) = PM.Trs(3,4,end);    
    X = [x0]; 
    Y = [y0]; 
    Z = [z0];

    clf;
    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(X,Y,Z, 'color', 'red', 'Linewidth', 0.5);  

    subplot(1,2,2) 
    plot3(X,Y,Z, 'color', 'Black', 'Linewidth', 0.5);  
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;
end

% 2nd Path
x_d = pos(:, 3);
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; diff = 10; dt = 0.4;epsilon = 0.4;

while diff > epsilon && k < 100  

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

    x1(k) = PM.Trs(1,4,end); y1(k) = PM.Trs(2,4,end); z1(k) = PM.Trs(3,4,end);
    X = [x0,x1];
    Y = [y0,y1];
    Z = [z0,z1];

    clf;
    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(X, Y, Z,'color', 'red', 'Linewidth', 0.5)  

    subplot(1,2,2)
    plot3(X, Y, Z,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

end

% 3rd Path 
x_d = pos(:, 4);
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; diff = 10; dt = 0.4;epsilon = 0.4;

while diff > epsilon && k < 100  

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

    x2(k) = PM.Trs(1,4,end); y2(k) = PM.Trs(2,4,end); z2(k) = PM.Trs(3,4,end);
    X = [x0,x1,x2];
    Y = [y0,y1,y2];
    Z = [z0,z1,z2];

    clf;
    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(X, Y, Z,'color', 'red', 'Linewidth', 0.5)  

    subplot(1,2,2)
    plot3(X, Y, Z,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;
end