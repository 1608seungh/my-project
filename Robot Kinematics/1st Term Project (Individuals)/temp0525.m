% Robot Kinematic 1st Term Project

clear all; clc; clf;

PM = Func_Group18;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0], [0;0;1]];
q = [[20;-20;0], [20;-20;10], [20;-50;10], [20;-70;10], [20; -80; 10], [20;-95;10]];
M_end = [eye(3), [20; -100; 10]; 0 0 0 1];

% 1st Path [시작위치 : [52; 25; 40]]
x_d =[52; -25; 40];
T_sd = [eye(3), x_d; 0 0 0 1]; 
th_deg = [0; pi/2; pi/4; pi/3; 0; 0];
th = deg2rad(th_deg);

k = 1;  

dt = 0.4;
epsilon = 0.1;
diff = 10;

while diff > epsilon && k < 500  

    PM.T_sd = T_sd; PM.w = w; PM.q = q;
    PM.M_end = M_end; PM.th = th; PM = PM.calculate;

    T_sb = PM.Trs(:,:,end);
    x = T_sb(1:3, 4);
    diff = norm(x_d-x);

    x_db = inv(T_sb) * [x_d; 1];
    x_db = x_db(1:3);

    v_b = x_db/norm(x_db)*0.5;
    Jbv = PM.Jb(4:6, :);
    dth = pinv(Jbv) * v_b;
    th = th + dth * dt;
    th_deg = rad2deg(th)';

    x0(k) = PM.Trs(1,4,end); y0(k) = PM.Trs(2,4,end); z0(k) = PM.Trs(3,4,end);
    
    X = x0; Y = y0; Z = z0;

%     clf;
%     subplot(1,2,1)
%     PM = PM.draw_xyz;
%     plot3(X,Y,Z, 'color', 'red', 'Linewidth', 0.5);  
% %     plot3(x0, y0, z0,'color', 'red', 'Linewidth', 0.5);  
% 
%     subplot(1,2,2) 
%     plot3(X,Y,Z, 'color', 'Black', 'Linewidth', 0.5);  
% %    plot3(x0, y0, z0,'color', 'black', 'Linewidth', 0.5) 
%     view([0,0,1])
%     PM.draw_xy;
% 
%     pause(0.001);
    k = k+1;
end
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
