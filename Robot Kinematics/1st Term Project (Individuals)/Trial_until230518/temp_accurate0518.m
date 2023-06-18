% Robot Kinematic 1st Term Project

clear all; clc; clf;

PM = Term_Project_Classdef;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0], [0;0;1]];
q = [[20;20;0], [20;20;10], [20;50;10], [20;70;10], [20; 80; 10], [20;95;10]];
M_end = [eye(3), [20; 100; 10]; 0 0 0 1];

% 1st Path [시작위치 : [52; 25; 40]]
x_d =[33; 35; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 
th_deg = [-89.9844; 115.3281; -107.0494; -8.2786; -0.0000; 89.9844];
th = deg2rad(th_deg);

k = 1;  
PM.T_sd = T_sd; PM.w = w; PM.q = q;
PM.M_end = M_end; PM.th = th; PM = PM.calculate;

while (norm(PM.w_b) > 0.07 || norm(PM.v_b) > 0.03) && k < 150  
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

% 2nd 
x_d =[15; 26; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.07 || norm(PM.v_b) > 0.03) && k < 150
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

% 3rd
x_d =[15; 35; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x2(k) = PM.Trs(1, 4, 7);  y2(k) = PM.Trs(2, 4, 7);  z2(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0, x1,y1,z1, x2,y2,z2, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0, x1,y1,z1, x2,y2,z2,'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 4th 
x_d =[22; 41; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x3(k) = PM.Trs(1, 4, 7);  y3(k) = PM.Trs(2, 4, 7);  z3(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 5th
x_d =[35; 42; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x4(k) = PM.Trs(1, 4, 7);  y4(k) = PM.Trs(2, 4, 7);  z4(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 6th 
x_d =[46; 40; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x5(k) = PM.Trs(1, 4, 7);  y5(k) = PM.Trs(2, 4, 7);  z5(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 7th 
x_d =[53; 35; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x6(k) = PM.Trs(1, 4, 7);  y6(k) = PM.Trs(2, 4, 7);  z6(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 8th 
x_d =[52; 25; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x7(k) = PM.Trs(1, 4, 7);  y7(k) = PM.Trs(2, 4, 7);  z7(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 9th
x_d =[66; 18; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x8(k) = PM.Trs(1, 4, 7);  y8(k) = PM.Trs(2, 4, 7);  z8(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 10th 
x_d =[34; 1; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x9(k) = PM.Trs(1, 4, 7);  y9(k) = PM.Trs(2, 4, 7);  z9(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 11th 
x_d =[1; 18; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x10(k) = PM.Trs(1, 4, 7);  y10(k) = PM.Trs(2, 4, 7);  z10(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 12th 
x_d =[8; 23; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x11(k) = PM.Trs(1, 4, 7);  y11(k) = PM.Trs(2, 4, 7);  z11(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 13th 
x_d =[15; 26; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x12(k) = PM.Trs(1, 4, 7);  y12(k) = PM.Trs(2, 4, 7);  z12(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12,'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 14th
x_d =[8; 23; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x13(k) = PM.Trs(1, 4, 7);  y13(k) = PM.Trs(2, 4, 7);  z13(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 15th
x_d =[7; 30; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x14(k) = PM.Trs(1, 4, 7);  y14(k) = PM.Trs(2, 4, 7);  z14(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 16th 
x_d =[11; 41; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x15(k) = PM.Trs(1, 4, 7);  y15(k) = PM.Trs(2, 4, 7);  z15(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15,'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 17th 
x_d =[4; 42; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x16(k) = PM.Trs(1, 4, 7);  y16(k) = PM.Trs(2, 4, 7);  z16(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 18th 
x_d =[7; 30; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x17(k) = PM.Trs(1, 4, 7);  y17(k) = PM.Trs(2, 4, 7);  z17(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 19th 
x_d =[8; 23; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x18(k) = PM.Trs(1, 4, 7);  y18(k) = PM.Trs(2, 4, 7);  z18(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, x18,y18,z18, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, x18,y18,z18, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 20th 
x_d =[22; 19; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x19(k) = PM.Trs(1, 4, 7);  y19(k) = PM.Trs(2, 4, 7);  z19(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, x18,y18,z18, x19,y19,z19, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, x18,y18,z18, x19,y19,z19, 'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end

% 21st 
x_d =[34; 17; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

while (norm(PM.w_b) > 0.05 || norm(PM.v_b) > 0.01) && k < 150
    clf;

    x20(k) = PM.Trs(1, 4, 7);  y20(k) = PM.Trs(2, 4, 7);  z20(k) = PM.Trs(3, 4, 7); 
    
    subplot(1,2,1)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, x18,y18,z18, x19,y19,z19, x20,y20,z20, 'color', 'red', 'Linewidth', 0.5)
    hold on;
    PM.draw_xyz;

    subplot(1,2,2)
    plot3(x0,y0,z0,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, ...
        x8,y8,z8, x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, ...
        x15,y15,z15, x16,y16,z16, x17,y17,z17, x18,y18,z18, x19,y19,z19, x20,y20,z20,'color', 'Black', 'Linewidth', 0.5)
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM = PM.calculate;
end
