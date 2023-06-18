clear all; clc; clf;
PM = PM_Func_0526;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0; 0; 0.1253], [0; 0.0834; 0.1253], [0; 0.1668; 0.1253], [0; 0.1668; 0.0753]];
M_end = [eye(3), [0; 0.1668; 0.0479]; 0, 0, 0, 1];

% Initial Pos = [0; 0.12; 0.0665]
th = [0.0315; 52.0915; -86.5612; 34.4697; -0.0315];
th = deg2rad(th);

j = 1; PM.w = w; PM.q = q; PM.M_end = M_end;
v_b = [1;1;1]; w_b = [1;1;1];

pos_d = [[0.075; 0.12; 0.0665], [0; 0.12; 0.0665], [-0.075; 0.12; 0.0665], [0; 0.12; 0.0665]];

for k = 1:1:4
    x_d = pos_d(:,k);
    T_sd = [eye(3), x_d; 0 0 0 1];

    j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 1;

    while (norm(w_b) > 0.001 || norm(v_b) > 0.0003) && j < 300   
        
        PM.T_sd = T_sd;  PM.th = th; PM = PM.calculate; PM.T_sb;           
        w_b = PM.V_b(1:3); v_b = PM.V_b(4:6);

        dth = pinv(PM.Jb)*PM.V_b;
        th = th + dth*dt;
        th_deg = rad2deg(th)';

        % Position 
        Xb(j,k) = PM.Trs(1,4,end); Yb(j,k) = PM.Trs(2,4,end); Zb(j,k) = PM.Trs(3,4,end); 
        Xb = nonzeros(Xb(:)); Yb = nonzeros(Yb(:)); Zb = nonzeros(Zb(:));
        pos = [Xb, Yb, Zb];

        % Raspi Robot arm Angle
        th1(j,k) = th_deg(1)+90;
        th2(j,k) = th_deg(2);
        th3(j,k) = th_deg(3)+90;
        th4(j,k) = th_deg(4);
        th5(j,k) = th_deg(5)+90;

        th1 = nonzeros(th1(:));
        th2 = nonzeros(th2(:));
        th3 = nonzeros(th3(:));
        th4 = nonzeros(th4(:));
        th5 = nonzeros(th5(:));

        j = j+1;
    end
end

err_y = abs(x_d(2) - Yb) / x_d(2) * 100;
abs_err = abs(x_d(2) - Yb);
Max_err_y = max(err_y);
Max_err = max(abs_err);
fprintf("Max_err_rel is %.4f%%, Max_error_abs is %.4fm", Max_err_y, Max_err)
disp(" ")

max1 = max(th1); min1 = min(th1);
max2 = max(th2); min2 = min(th2);
max3 = max(th3); min3 = min(th3);
max4 = max(th4); min4 = min(th4);
max5 = max(th5); min5 = min(th5);

Max_servo_angle = [max1, max2, max3, max4, max5]
min_servo_angle = [min1, min2, min3, min4, min5]