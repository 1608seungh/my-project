clear all; clc; clf;

PM = Func_0522temp;
% ACT = Func_math20230515;

% Desired Pos & Orient
x_d =[0.2; 0.1; 0.15];
T_sd = [eye(3), x_d; 0 0 0 1]; 

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0; 0; 0.0701], [0; 0.1036; 0.0701], [0; 0.2004; 0.0701], [-0.0094; 0.2266; 0.0391]];
M_end = [eye(3), [-0.0094; 0.2266; 0.0068]; 0 0 0 1];
th = [0; pi/12; -pi/6; pi/12; 0]; % Initial Angle
th_deg = rad2deg(th)'

k = 1; PM.T_sd = T_sd; PM.w = w; PM.q = q;
v_b = [1;1;1]; w_b = [1;1;1];

while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && k < 200
    clf;

    PM.M_end = M_end; PM.th = th; PM = PM.calculate;

    v_b = PM.V_b(4:6);
    w_b = PM.V_b(1:3);
% 
%     dth = pinv(PM.Jb) * PM.V_b;
%     th = th + dth;


%     Jbv = PM.Jb(4:6,:);
%     gamma = 0.2;
    th_home = [0; pi/12; -pi/6; pi/12; 0];
    dth = pinv(PM.Jb)*PM.V_b + (eye(5)-pinv(PM.Jb)*PM.Jb) * gamma * (th_home-th);
    th = th + dth;
    th_deg = rad2deg(th)' 

    PM = PM.draw_linkage;
    
    xx(k) = PM.q_(1,end); yy(k) = PM.q_(2,end); zz(k) = PM.q_(3,end); 
    T_sb = PM.Trs(:,:,end)
    plot3(xx,yy,zz, 'color', 'red', 'linewidth', 0.5)

    pause(0.001);

    k = k+1

end