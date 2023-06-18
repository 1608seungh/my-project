clear all; clc; clf;

PM = Func_0522temp;

% Data Needed to Calculate 
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0; 0; 0.0701], [0; 0.1036; 0.0701], [0; 0.2004; 0.0701], [-0.0094; 0.2266; 0.0391]];
M_end = [eye(3), [-0.0094; 0.2266; 0.0068]; 0 0 0 1];

th = [-86.9474; 41.3064; -44.3338; 3.0274; 86.9474]; % Initial Angle
ard_th = [th(1); th(2); -th(3); -th(4); th(5)] + [90; 0; 90; 180; 90];
ard_th_digital = ard_th / 180;
th = deg2rad(th);

j = 1; PM.w = w; PM.q = q; PM.M_end = M_end;
v_b = [1;1;1]; w_b = [1;1;1];
pos = [[0.15; 0.11; 0.0701], [0.13; 0.14; 0.0701], [0.13; 0.09; 0.0701], [0.16; 0.09; 0.0701], [0.13; 0.09; 0.0701], [0.19; 0.03; 0.0701]];

% Arduino Setting
a = arduino();

s1 = servo(a,"D2", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s2 = servo(a,"D3", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s3 = servo(a,"D9", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s4 = servo(a,"D8", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s5 = servo(a,"D4", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);

writePosition(s1, ard_th_digital(1));
writePosition(s2, ard_th_digital(2));
writePosition(s3, ard_th_digital(3));
writePosition(s4, ard_th_digital(4));
writePosition(s5, ard_th_digital(5));

for k = 1:1:6
    x_d = pos(:,k);
    T_sd = [eye(3), x_d; 0 0 0 1];

    j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 3;

    while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && j < 200
    
        PM.T_sd = T_sd;  PM.th = th; PM = PM.calculate;
    
        v_b = PM.V_b(4:6);
        w_b = PM.V_b(1:3);

        gamma = 0.2;
        th_home = [0; 0; 0; pi/4; 0];
        dth = pinv(PM.Jb)*PM.V_b + (eye(5)-pinv(PM.Jb)*PM.Jb) * gamma * (th_home-th);
        th = th + dth*dt;
        th_deg = rad2deg(th)' 
    

        ard_th = [th_deg(1); th_deg(2); -th_deg(3); -th_deg(4); th_deg(5)] + [90; 0; 90; 180; 90];
        ard_th_digital = ard_th / 180;

        writePosition(s1, ard_th_digital(1));
        writePosition(s2, ard_th_digital(2));
        writePosition(s3, ard_th_digital(3));
        writePosition(s4, ard_th_digital(4));
        writePosition(s5, ard_th_digital(5));


            PM.T_sb
        
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