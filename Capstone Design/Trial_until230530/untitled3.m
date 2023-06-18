clear all; clc; clf;

PM = Func_0525temp;

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0; 0; 0.1253], [0; 0.0834; 0.1253], [0; 0.1668; 0.1253], [0; 0.1668; 0.0753]];
M_end = [eye(3), [0; 0.1668; 0.0479]; 0, 0, 0, 1];

% Pos = [-0.075; 0.12; 0.0689]
th = [32.0054; 38.6918; -62.3099; 23.6180; -32.0054];
% th = [31.9677; 44.3471; -55.6161; 11.2690; -31.9616];
th = deg2rad(th);

j = 1; PM.w = w; PM.q = q; PM.M_end = M_end;
v_b = [1;1;1]; w_b = [1;1;1];
pos = [[0; 0.12; 0.0665], [0; 0.12; 0.0899], [-0.075; 0.12; 0.0899], [0.06; 0.12; 0.0899]];

for k = 1:1
    x_d = pos(:,k);
    T_sd = [eye(3), x_d; 0 0 0 1];

    j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 2;

    while (norm(w_b) > 0.0001 || norm(v_b) > 0.00001) && j < 300   
        PM.T_sd = T_sd;  PM.th = th; PM = PM.calculate; PM.T_sb;
    
        v_b = PM.V_b(4:6);
        w_b = PM.V_b(1:3);

        gamma = -0.2;

        th_center = zeros(5,1);
        th_center(3) = -pi/3;
        th_center(4) = pi/6;
        grad_H = zeros(5,1);
        grad_H(3) = th(3) - th_center(3);
        grad_H(4) = th(4) - th_center(4);

        Jvb = pinv(PM.Jb)*PM.V_b;
        dth = Jvb + gamma * (eye(5)-pinv(PM.Jb)*PM.Jb) * grad_H;
        th = th + dth*dt;
        th_deg = rad2deg(th)'
        thd(j) = th_deg(3);
        thd = thd(:);
 
        X(j,k) = PM.Trs(1,4,end); 
        Y(j,k) = PM.Trs(2,4,end);
        Z(j,k) = PM.Trs(3,4,end);
        
        Xb = nonzeros(X(:));
        Yb = nonzeros(Y(:));
        Zb = nonzeros(Z(:));
        
        clf
        subplot(1,2,1)
        plot3(Xb,Yb,Zb, 'color', 'red', 'Linewidth', 0.5);  
        hold on
        PM = PM.draw_xyz;
        
        subplot(1,2,2) 
        plot3(Xb,Yb,Zb, 'color', 'Black', 'Linewidth', 0.5);  
        view([0,0,1])
        PM.draw_xy;
        
        pause(0.001);
        
        j = j+1;

    end
end
