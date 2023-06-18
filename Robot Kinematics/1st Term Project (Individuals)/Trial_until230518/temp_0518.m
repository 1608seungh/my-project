% Robot Kinematic 1st Term Project

clear all; clc; clf;

PM = temp_Term_Project;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0], [0;0;1]];
q = [[20;-20;0], [20;-20;10], [20;-50;10], [20;-70;10], [20; -80; 10], [20;-95;10]];
M_end = [eye(3), [20; -100; 10]; 0 0 0 1];

% 1st Path [시작위치 : [52; 25; 40]]
x_d =[33; 35; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 
th_deg = [-89.9844; 115.3281; -107.0494; -8.2786; -0.0000; 89.9844];
th = deg2rad(th_deg);

k = 1;  

dt = 0.4;
epsilon = 0.4;
diff = 10;

while diff > epsilon && k < 100  

    PM.T_sd = T_sd; PM.w = w; PM.q = q;
    PM.M_end = M_end; PM.th = th; PM = PM.calculate;

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
    
    X = x0; Y = y0; Z = z0;

    clf;
    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(X,Y,Z, 'color', 'red', 'Linewidth', 0.5);  
%     plot3(x0, y0, z0,'color', 'red', 'Linewidth', 0.5);  

    subplot(1,2,2) 
    plot3(X,Y,Z, 'color', 'Black', 'Linewidth', 0.5);  
%    plot3(x0, y0, z0,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;
end

% 2nd Path
x_d =[15; 26; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; 

dt = 0.5;
epsilon = 0.5;
diff = 10;

while diff > epsilon && k < 100  

    PM.T_sd = T_sd; PM.th = th; PM = PM.calculate;
    
    T_sb = PM.Trs(:,:,end);
    x = T_sb(1:3, 4);
    diff = norm(x_d-x);

    x_db = inv(T_sb) * [x_d; 1];
    x_db = x_db(1:3);

    v_b = x_db/norm(x_db)*2;
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

    hold on;
    subplot(1,2,2)
    plot3(X, Y, Z,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

end

% 3rd Path 
x_d =[15; 35; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

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

% 4th 
x_d =[22; 41; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

% 5th 
x_d =[35; 42; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x4(k) = PM.Trs(1,4,end); y4(k) = PM.Trs(2,4,end); z4(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 6th 
x_d =[46; 40; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x5(k) = PM.Trs(1,4,end); y5(k) = PM.Trs(2,4,end); z5(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5,'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 7th 
x_d =[53; 35; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x6(k) = PM.Trs(1,4,end); y6(k) = PM.Trs(2,4,end); z6(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 8th
x_d =[52; 25; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x7(k) = PM.Trs(1,4,end); y7(k) = PM.Trs(2,4,end); z7(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 9th
x_d =[66; 18; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x8(k) = PM.Trs(1,4,end); y8(k) = PM.Trs(2,4,end); z8(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 10th
x_d =[34; 1; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x9(k) = PM.Trs(1,4,end); y9(k) = PM.Trs(2,4,end); z9(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 11th
x_d =[1; 18; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x10(k) = PM.Trs(1,4,end); y10(k) = PM.Trs(2,4,end); z10(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 12th
x_d =[8; 23; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x11(k) = PM.Trs(1,4,end); y11(k) = PM.Trs(2,4,end); z11(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 13th
x_d =[15; 26; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x12(k) = PM.Trs(1,4,end); y12(k) = PM.Trs(2,4,end); z12(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 14th 
x_d =[8; 23; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x13(k) = PM.Trs(1,4,end); y13(k) = PM.Trs(2,4,end); z13(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 15th 
x_d =[7; 30; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x14(k) = PM.Trs(1,4,end); y14(k) = PM.Trs(2,4,end); z14(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 16th 
x_d =[11; 41; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x15(k) = PM.Trs(1,4,end); y15(k) = PM.Trs(2,4,end); z15(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 17th 
x_d =[4; 42; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x16(k) = PM.Trs(1,4,end); y16(k) = PM.Trs(2,4,end); z16(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 18th
x_d =[7; 30; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x17(k) = PM.Trs(1,4,end); y17(k) = PM.Trs(2,4,end); z17(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, ...
        x17,y17,z17, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, ...
        x17,y17,z17, 'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 19th 
x_d =[8; 23; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x18(k) = PM.Trs(1,4,end); y18(k) = PM.Trs(2,4,end); z18(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, ...
        x17,y17,z17, x18,y18,z18, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, ...
        x17,y17,z17, x18,y18,z18,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end

% 20th 
x_d =[34; 17; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 

k = 1; PM.T_sd = T_sd; PM = PM.calculate;

dt = 0.3;
epsilon = 0.4;
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

    x19(k) = PM.Trs(1,4,end); y19(k) = PM.Trs(2,4,end); z19(k) = PM.Trs(3,4,end);

    subplot(1,2,1)
    PM = PM.draw_xyz;
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, ...
        x17,y17,z17, x18,y18,z18, x19,y19,z19, 'color', 'red', 'Linewidth', 0.5)  
  
    subplot(1,2,2)
    plot3(x0,y0,z0, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, x5,y5,z5, x6,y6,z6, x7,y7,z7, x8,y8,z8, ...
        x9,y9,z9, x10,y10,z10, x11,y11,z11, x12,y12,z12, x13,y13,z13, x14,y14,z14, x15,y15,z15, x16,y16,z16, ...
        x17,y17,z17, x18,y18,z18, x19,y19,z19,'color', 'black', 'Linewidth', 0.5) 
    view([0,0,1])
    PM.draw_xy;

    pause(0.001);
    k = k+1;

    PM.th = th;
    PM = PM.calculate;
end