clear all; clc; 

ACT = Func20230403;

% Desired Position & Orientation of End Effector 
x_d = [0.1; 0.1; -0.1];
% R_d = [cosd(30) sind(30) 0; -sind(30) cosd(30) 0; 0 0 1];
R_d = [1 0 0; 0 1 0; 0 0 1];
T_sd = [R_d, x_d; 0 0 0 1]; % 4x4 Matrix

% Zero Position
M01 = eye(4);
M02 = eye(4);
M03 = [eye(3), [0;0;0.1036]; 0 0 0 1];
M04 = [eye(3), [0;0;0.2011]; 0 0 0 1];
M05 = [eye(3), [-0.01;0;0.2280]; 0 0 0 1];
M = [eye(3), [-0.035;-0.081;0.228]; 0 0 0 1]; 

% Angular Velocity // 3x1 Matrix
w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; 
w4 = [1;0;0]; w5 = [0;-1;0]; w6 = [0;0;1];

% Point on the axis of the Screw (q) // 3x1 Matrix 
q1 = [0;0;0]; q2 = [0;0;0]; q3 = [0;0;0.1036];
q4 = [0;0;0.2011]; q5 = [-0.01;0;0.2280]; q6 = [-0.025; -0.081; 0];

% Skew-symmetric Matrix Representation of w // 3x3 Matrix
w_mat1 = ACT.Skew(w1); w_mat2 = ACT.Skew(w2); w_mat3 = ACT.Skew(w3); 
w_mat4 = ACT.Skew(w4); w_mat5 = ACT.Skew(w5); w_mat6 = ACT.Skew(w6); 

% Linear Velocity // 3x1 Matrix 
v1 = -w_mat1*q1; v2 = -w_mat2*q2; v3 = -w_mat3*q3; 
v4 = -w_mat4*q4; v5 = -w_mat5*q5; v6 = -w_mat6*q6;

% Screw Axis // 4x4 Matrix // [w_mat, v; 0 0 0 0]
S1 = ACT.Screw(w1,q1); S2 = ACT.Screw(w2,q2); S3 = ACT.Screw(w3,q3); 
S4 = ACT.Screw(w4,q4); S5 = ACT.Screw(w5,q5); S6 = ACT.Screw(w6,q6);

% Initial Angle 
i = 0;
th1 = 0; th2  = 0; th3 = pi/9; th4 = 0; th5 = 0; th6 = 0;
th = [th1; th2; th3; th4; th5; th6] % 6x1 Matrix 

% Tranformation Matrix (Space(fixed) to Body Frame)
T01 = expm(S1*th(1))*M01;
T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
T05 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05;
T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M
T_bs = inv(T_sb);

% Jacobian Matrix // 6x6 Matrix
Js = [w1,w2,w3,w4,w5,w6; v1,v2,v3,v4,v5,v6]; % Initial Space Jacobian Matrix; Jsi = (w_i; v_i)
Jb = ACT.Ad(T_bs)*Js; % Initial Body Jacobian Matrix / 6x6 Matrix

% Numerical Inverse Kinematic 
T_bd = T_bs*T_sd; % 4x4 Matrix 
V_bmat = logm(T_bd)*0.08; % 4x4 Matrix
V_b = ACT.BodyTwist(V_bmat); % V_b = [w_bi; v_bi]; 6x1 Matrix 
w_b = V_b(1:3); 
v_b = V_b(4:6);

while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && i < 1000
    
    dth = pinv(Jb)*V_b;
    th = th + dth; % 6x1 Matrix (rad)
    th_degree = rad2deg(th)'

    i = i+1

    T01 = expm(S1*th(1))*M01;
    T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
    T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
    T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
    T05 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05;
    T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M
    T_bs = inv(T_sb); 
    xx(i) = T_sb(1,4); yy(i) = T_sb(2,4); zz(i) = T_sb(3,4);

    Js1 = ACT.S2twist(S1);
    Js2 = ACT.Ad(expm(S1*th(1))) * ACT.S2twist(S2);
    Js3 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))) * ACT.S2twist(S3);
    Js4 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))) * ACT.S2twist(S4);
    Js5 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))) * ACT.S2twist(S5);
    Js6 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))) * ACT.S2twist(S6);
    Js = [Js1, Js2, Js3, Js4, Js5, Js6];
    Jb = ACT.Ad(T_bs)*Js; 
    
    T_bd = T_bs*T_sd; % 4x4 Matrix 
    V_bmat = logm(T_bd)*0.08; % 4x4 Matrix
    V_b = ACT.BodyTwist(V_bmat); % V_b = [w_b; v_b]; 6x1 Matrix
    w_b = V_b(1:3)'
    v_b = V_b(4:6)'

    x0 = 0; y0 = 0; z0 = 0;
    x1(i) = T01(1,4); y1(i) = T01(2,4); z1(i) = T01(3,4);
    x2(i) = T02(1,4); y2(i) = T02(2,4); z2(i) = T02(3,4);
    x3(i) = T03(1,4); y3(i) = T03(2,4); z3(i) = T03(3,4);
    x4(i) = T04(1,4); y4(i) = T04(2,4); z4(i) = T04(3,4);
    x5(i) = T05(1,4); y5(i) = T05(2,4); z5(i) = T05(3,4);
    xx(i) = T_sb(1,4); yy(i) = T_sb(2,4); zz(i) = T_sb(3,4);
   

    p_1 = T01(1:3,4); p_2 = T02(1:3,4); p_3 = T03(1:3,4); 
    p_4 = T04(1:3,4); p_5 = T05(1:3,4); p_6 = T_sb(1:3,4);
    line([p_1(1), p_2(1)], [p_1(2), p_2(2)], [p_1(3), p_2(3)], 'Color', 'Magenta','LineWidth', 2)
    line([p_2(1), p_3(1)], [p_2(2), p_3(2)], [p_2(3), p_3(3)], 'Color', 'Cyan', 'LineWidth', 2)
    line([p_3(1), p_4(1)], [p_3(2), p_4(2)], [p_3(3), p_4(3)], 'Color', 'black', 'LineWidth', 1)
    line([p_4(1), p_5(1)], [p_4(2), p_5(2)], [p_4(3), p_5(3)], 'LineWidth', 2)
    line([p_5(1), p_6(1)], [p_5(2), p_6(2)], [p_5(3), p_6(3)], 'LineWidth', 2)
%     view(3)


end

    T01 = expm(S1*th(1))*M01;
    T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
    T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
    T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
    T05 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05;
    T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M;

% plot3(xx,yy,zz,'color', 'red', 'LineWidth', 1)

hold on
plot3(x0,y0,z0,'o', 'color', 'Black', 'LineWidth', 2.5)
plot3(x1,y1,z1, 'color', 'black', 'LineWidth', 0.1) 
plot3(x2,y2,z2, 'color', 'Blue', 'LineWidth', 0.1) 
plot3(x3,y3,z3, 'color', 'green', 'LineWidth', 0.1) 
plot3(x4,y4,z4, 'color', 'cyan', 'LineWidth', 0.1) 
plot3(x5,y5,z5, 'color', 'magenta', 'LineWidth', 0.1) 
plot3(xx,yy,zz, 'color', 'red', 'LineWidth', 1)
view(3)
hold off


xlabel('x'); ylabel('y'); zlabel('z');
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([-0.15 0.3])



grid on
hold off