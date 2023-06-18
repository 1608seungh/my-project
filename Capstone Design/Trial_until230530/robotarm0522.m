clear all; clc; clf;

PM = Func_0522;
% ACT = Func_math20230515;

% Desired Pos & Orient
x_d =[0.2; 0.1; 0.15];
T_sd = [eye(3), x_d; 0 0 0 1]; 

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0]];
q = [[0;0;0], [0;0;0], [0;0;0.1036], [0;0;0.2011], [0; 0.081; 0.2011]];
% q = [[0;0;0], [0;0;0], [0;0;0.1036], [0;0;0.2011], [0.015; 0.081; 0.2280]];
M_end = [eye(3), [0; 0.1516; 0.2011]; 0 0 0 1];
th = [-pi/4; -pi/6; -pi/18; pi/6; pi/4]; % Initial Angle
th_deg = rad2deg(th)'

PM.w = w; PM.q = q; PM.M_end = M_end;

k = 1;
v_b = [1;1;1]; 

while norm(v_b) > 0.0001 && k < 200
    
    clf;
    PM.T_sd = T_sd; PM.th = th; PM = PM.calculate;

    v_b = PM.V_b(4:6);
%     w_b = PM.V_b(1:3);

%     dth = pinv(PM.Jb) * PM.V_b;
%     th = th + dth;

    Jbv = PM.Jb(4:6,:);
    gamma = 0.2;
    th_home = [-pi/4; -pi/6; -pi/18; pi/6; 0];
    dth = pinv(Jbv)*v_b + (eye(5)-pinv(Jbv)*Jbv) * gamma * (th_home-th);
    th = th + dth;
%     th_deg = rad2deg(th)' 

    X(k) = PM.Trs(1,4,end); 
    Y(k) = PM.Trs(2,4,end);
    Z(k) = PM.Trs(3,4,end);

    Xb = nonzeros(X(:));
    Yb = nonzeros(Y(:));
    Zb = nonzeros(Z(:));

    subplot(1,2,1)
    PM = PM.draw_xyz;
    hold on
    plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5); 
    
    
    subplot(1,2,2) 
    plot3(Xb,Yb,Zb, 'color', 'Black', 'Linewidth', 0.5);  
    view([0,0,1])
    PM.draw_xy;
% 
% 
% 
%     plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5);  
%     hold on
%     PM = PM.draw_xyz;
    
%     xx(k) = PM.q_(1,end); yy(k) = PM.q_(2,end); zz(k) = PM.q_(3,end); 
%     T_sb = PM.Trs(:,:,end)
%     plot3(xx,yy,zz, 'color', 'red', 'linewidth', 0.5)

    pause(0.001);

    k = k+1;

end