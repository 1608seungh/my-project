clear all; clc; 

ACT = Func20230403;
rot = rotations;
tra = transformations;

% Desired Position & Orientation of End Effector 
x_d = [0.1; 0.1; 0.15];
R_d = [cosd(30) -sind(30) 0; sind(30) cosd(30) 0; 0 0 1];
T_sd = [R_d, x_d; 0 0 0 1]; % 4x4 Matrix

% Zero Position
M01 = [eye(3), [0; 0; 0.0566]; 0 0 0 1]; % 2nd Joint Pos & Orient
M02 = [eye(3), [0; 0.1036; 0.0566]; 0 0 0 1]; % 3nd Joint Pos & Orient
M03 = [eye(3), [0; 0.2012; 0.0566]; 0 0 0 1]; % 4th Joint Pos & Orient
M04 = [eye(3), [0.0097; 0.2012; 0.0875]; 0 0 0 1]; % 5th Joint Pos & Orient
M05 = [eye(3), [0.0097; 0.3583; 0.0875]; 0 0 0 1]; % End-Effector Joint Pos & Orient

% Angular Velocity 
w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;1;0];

% Point on the axis of the Screw (q) 
q1 = [0; 0; 0]; q2 = [0; 0; 0.0566]; q3 = [0; 0.1036; 0.0566];
q4 = [0; 0.2012; 0.0566]; q5 = [0.0097; 0.2012; 0.0875];

% Linear Velocity // 3x1 Matrix 
v1 = -cross(w1,q1); v2 = -cross(w2,q2); v3 = -cross(w3,q3); 
v4 = -cross(w4,q4); v5 = -cross(w5,q5); 

% Screw Axis // 4x4 Matrix
S1 = ACT.Screw(w1,q1); S2 = ACT.Screw(w2,q2); S3 = ACT.Screw(w3,q3); 
S4 = ACT.Screw(w4,q4); S5 = ACT.Screw(w5,q5); 

% Initial Angle 
i = 0;
th = [-pi/2; pi/2; -pi/2; 0; 0];

% Tranformation Matrix 
T01 = expm(S1*th(1))*M01;
T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05
T_bs = inv(T_sb);

% Jacobian Matrix // 6x6 Matrix
Js1 = ACT.S2twist(S1);
Js2 = ACT.Ad(expm(S1*th(1))) * ACT.S2twist(S2);
Js3 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))) * ACT.S2twist(S3);
Js4 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))) * ACT.S2twist(S4);
Js5 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))) * ACT.S2twist(S5);
Js = [Js1, Js2, Js3, Js4, Js5];
Jb = ACT.Ad(T_bs)*Js; % Initial Body Jacobian Matrix / 6x6 Matrix

% Numerical Inverse Kinematic 
T_bd = T_bs*T_sd; % 4x4 Matrix 
V_bmat = logm(T_bd)*0.08; % 4x4 Matrix
V_b = ACT.BodyTwist(V_bmat); % V_b = [w_bi; v_bi]; 6x1 Matrix 
w_b = V_b(1:3) 
v_b = V_b(4:6)

while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && i < 150
    
    dth = pinv(Jb)*V_b;
    th = th + dth;
    th_deg = rad2deg(th)'

    i = i+1

    T01 = expm(S1*th(1))*M01;
    T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
    T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
    T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
    T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05
    T_bs = inv(T_sb);

    Js1 = ACT.S2twist(S1);
    Js2 = ACT.Ad(expm(S1*th(1))) * ACT.S2twist(S2);
    Js3 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))) * ACT.S2twist(S3);
    Js4 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))) * ACT.S2twist(S4);
    Js5 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))) * ACT.S2twist(S5);
    Js = [Js1, Js2, Js3, Js4, Js5];
    Jb = ACT.Ad(T_bs)*Js;
    
    T_bd = T_bs*T_sd; % 4x4 Matrix 
    V_bmat = logm(T_bd)*0.08; % 4x4 Matrix
    V_b = ACT.BodyTwist(V_bmat); % V_b = [w_b; v_b]; 6x1 Matrix
    w_b = V_b(1:3);
    v_b = V_b(4:6);

    x0 = 0; y0 = 0; z0 = 0;
    x1(i) = T01(1,4); y1(i) = T01(2,4); z1(i) = T01(3,4);
    x2(i) = T02(1,4); y2(i) = T02(2,4); z2(i) = T02(3,4);
    x3(i) = T03(1,4); y3(i) = T03(2,4); z3(i) = T03(3,4);
    x4(i) = T04(1,4); y4(i) = T04(2,4); z4(i) = T04(3,4);
    x5(i) = T_sb(1,4); y5(i) = T_sb(2,4); z5(i) = T_sb(3,4);

    p_1 = T01(1:3,4); p_2 = T02(1:3,4); p_3 = T03(1:3,4); 
    p_4 = T04(1:3,4); p_5 = T_sb(1:3,4);

    clf;
    plot3(x0,y0,z0,'x', 'color', 'black', 'Linewidth', 1)
    hold on;
    plot3(x5,y5,z5,'color', 'red', 'Linewidth', 0.5)

    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-0.3 0.3])
    ylim([-0.3 0.3])
    zlim([0 0.3])

    line([p_1(1), p_2(1)], [p_1(2), p_2(2)], [p_1(3), p_2(3)])
    line([p_2(1), p_3(1)], [p_2(2), p_3(2)], [p_2(3), p_3(3)])
    line([p_3(1), p_4(1)], [p_3(2), p_4(2)], [p_3(3), p_4(3)])
    line([p_4(1), p_5(1)], [p_4(2), p_5(2)], [p_4(3), p_5(3)], 'color', 'black', 'linewidth', 1)
    hold off;

    pause(0.0001)
    
end