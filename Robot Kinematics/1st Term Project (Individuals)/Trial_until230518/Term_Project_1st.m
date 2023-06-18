% Robot Kinematic 1st Term Project

clear all; clc; clf;

PM = Term_Project_Classdef;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0], [0;0;1]];
q = [[20;20;0], [20;20;10], [20;50;10], [20;70;10], [20; 80; 10], [20;95;10]];
M_end = [eye(3), [20; 100; 10]; 0 0 0 1];

% initial Path [시작위치 : [15; 45; 40]]
x_d =[30; 30; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 
th_deg = [14.0279; 137.0325; -108.4757; -28.5569; 0; -14.0279];
th = deg2rad(th_deg);

k = 1;  
PM.T_sd = T_sd; PM.w = w; PM.q = q;
PM.M_end = M_end; PM.th = th; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150  
    clf;    
    x0(k) = PM.Trs(1,4,end); y0(k) = PM.Trs(2,4,end); z0(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    plot3(x0,y0,z0,'color', 'red', 'Linewidth', 0.5)  
    hold on;
    PM.draw_xyz;
    
    subplot(1,2,2)
    plot3(x0,y0,z0,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end


% 1st Path 
x_d =[45; 45; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x1(k) = PM.Trs(1, 4, 7);  y1(k) = PM.Trs(2, 4, 7);  z1(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1,'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1,'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 5th Path
x_d =[30; 60; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;
    
    x2(k) = PM.Trs(1, 4, 7);  y2(k) = PM.Trs(2, 4, 7);  z2(k) = PM.Trs(3, 4, 7); 

    subplot(1,2,1)
    PM.draw_xyz;
    plot3(x0,y0,z0,x1,y1,z1,x2,y2,z2,'color', 'red', 'Linewidth', 0.5)

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1,x2,y2,z2,'color', 'black', 'Linewidth', 0.5)
    view([0,0,1])
%     plot(x2,y2, 'color', 'Black', 'Linewidth', 0.5)
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 4th Path
x_d =[15; 45; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;
    
    x3(k) = PM.Trs(1, 4, 7);  y3(k) = PM.Trs(2, 4, 7);  z3(k) = PM.Trs(3, 4, 7); 

    subplot(1,2,1)
    PM.draw_xyz;
    plot3(x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3,'color', 'red', 'Linewidth', 0.5)

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3,'color', 'black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end
