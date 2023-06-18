clear all; clc; clf;

PM = Func_0525temp;

w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;0;1]];
q = [[0;0;0], [0; 0; 0.1253], [0; 0.0834; 0.1253], [0; 0.1668; 0.1253], [0; 0.1668; 0.0753]];
M_end = [eye(3), [0; 0.1668; 0.0479]; 0, 0, 0, 1];

% Initial Angle at [0; 0.12; 0.0665]
th = [0.0315; 52.0915; -86.5612; 34.4697; -0.0315]; 
th = deg2rad(th);

PM.w = w; PM.q = q; PM.M_end = M_end;
pos_d = [[0.075; 0.12; 0.0665], [0; 0.12; 0.0665], [-0.075; 0.12; 0.0665], [0; 0.12; 0.0665]];

for k = 1:1:4
    x_d = pos_d(:,k);
    T_sd = [eye(3), x_d; 0 0 0 1];

    j = 1; v_b = [1;1;1]; w_b = [1;1;1]; dt = 1.5;

    while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && j < 300   
        
        PM.T_sd = T_sd;  PM.th = th; PM = PM.calculate; PM.T_sb;           
        w_b = PM.V_b(1:3); v_b = PM.V_b(4:6);

        dth = pinv(PM.Jb)*PM.V_b;
        th = th + dth*dt;
        th_deg = rad2deg(th)'
        
        j = j+1;
    end
end
