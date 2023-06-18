% Robot Kinematic 1st Term Project

clear all; clc; clf;

PM = temp_Term_Project;

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

dt = 0.3;
epsilon = 0.4;
diff = 10;

while diff > epsilon && k < 100  
    clf;
    
    T_sb = PM.Trs(:,:,end);
    x = T_sb(1:3, 4)
    diff = norm(x_d-x)

    x_db = inv(T_sb) * [x_d; 1];
    x_db = x_db(1:3);

    v_b = x_db/norm(x_db)*5;
    Jbv = PM.Jb(4:6, :);
    dth = pinv(Jbv) * v_b;
    th = th + dth * dt;

    x0(k) = PM.Trs(1,4,end); y0(k) = PM.Trs(2,4,end); z0(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0,'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

p = 0
% 1st Path 
x_d =[45; 45; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.5;
epsilon = 0.45;
diff = 10;

while diff > epsilon && k < 100  
    clf;
    
    T_sb = PM.Trs(:,:,end);
    x = T_sb(1:3, 4)
    diff = norm(x_d-x)

    x_db = inv(T_sb) * [x_d; 1];
    x_db = x_db(1:3);

    v_b = x_db/norm(x_db)*2;
    Jbv = PM.Jb(4:6, :);
    dth = pinv(Jbv) * v_b;
    th = th + dth * dt;

    x1(k) = PM.Trs(1,4,end); y1(k) = PM.Trs(2,4,end); z1(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 5th Path
x_d =[30; 60; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.6;
diff = 10;

while diff > epsilon && k < 100  
    clf;
    
    T_sb = PM.Trs(:,:,end);
    x = T_sb(1:3, 4);
    diff = norm(x_d-x);

    x_db = inv(T_sb) * [x_d; 1];
    x_db = x_db(1:3);

    v_b = x_db/norm(x_db)*3;
    Jbv = PM.Jb(4:6, :);
    dth = pinv(Jbv) * v_b;
    th = th + dth * dt;

    x2(k) = PM.Trs(1,4,end); y2(k) = PM.Trs(2,4,end); z2(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 4th Path
x_d =[15; 45; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.5;
epsilon = 0.5;
diff = 10;

while diff > epsilon && k < 100  
    clf;
    
    T_sb = PM.Trs(:,:,end);
    x = T_sb(1:3, 4);
    diff = norm(x_d-x);

    x_db = inv(T_sb) * [x_d; 1];
    x_db = x_db(1:3);

    v_b = x_db/norm(x_db)*4;
    Jbv = PM.Jb(4:6, :);
    dth = pinv(Jbv) * v_b;
    th = th + dth * dt;

    x3(k) = PM.Trs(1,4,end); y3(k) = PM.Trs(2,4,end); z3(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end