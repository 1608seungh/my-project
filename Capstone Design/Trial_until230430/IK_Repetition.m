clear all; clc; 

% Desired Position & Orientation of End Effector (원하는 위치값 입력) 
x_d = [-0.1; -0.1; 0.2]; 
R_d = [cosd(45) sind(45) 0; -sind(45) cosd(45) 0; 0 0 1];
T_sd = [R_d, x_d; 0 0 0 1]; % 4x4 Matrix

% Zero Position of Robot-Arm (기준이 되는 End-Effector의 현재 위치)
M = [eye(3), [-0.035;-0.081;0.228]; 0 0 0 1]; 

% Angular Velocity // 3x1 Matrix
w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;-1;0]; w6 = [0;0;1];

% Point on the axis of the Screw (q) // 3x1 Matrix 
q1 = [0;0;0]; q2 = [0;0;0]; q3 = [0;0;0.1036];
q4 = [0;0;0.2011]; q5 = [-0.01;0;0.2280]; q6 = [-0.025; -0.081; 0];

% Skew-symmetric Matrix Representation of w // 3x3 Matrix
w_mat1 = W(w1); w_mat2 = W(w2); w_mat3 = W(w3); w_mat4 = W(w4); w_mat5 = W(w5); w_mat6 = W(w6); 

% Linear Velocity // 3x1 Matrix 
v1 = -w_mat1*q1; v2 = -w_mat2*q2; v3 = -w_mat3*q3; v4 = -w_mat4*q4; v5 = -w_mat5*q5; v6 = -w_mat6*q6;

% Screw Axis // 4x4 Matrix // [w_mat, v; 0 0 0 0]
S1 = screw(w1,q1); S2 = screw(w2,q2); S3 = screw(w3,q3); S4 = screw(w4,q4); S5 = screw(w5,q5); S6 = screw(w6,q6);

% Initial Angle 
i = 0;
th1 = 0; th2 = 0; th3 = 0; th4 = 0; th5 = 0; th6 = 0;
th = [th1; th2; th3; th4; th5; th6]; % 6x1 Matrix 

% Tranformation Matrix (Space(fixed) to Body Frame)
T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M
T_bs = inv(T_sb);

% Jacobian Matrix // 6x6 Matrix
Js = [w1,w2,w3,w4,w5,w6; v1,v2,v3,v4,v5,v6] % Initial Space Jacobian Matrix; Jsi = (w_i; v_i)
Jb = ad(T_bs)*Js; % Initial Body Jacobian Matrix // 6x6 Matrix

% Numerical Inverse Kinematic 
T_bd = T_bs*T_sd; % 4x4 Matrix 
V_bmat = logm(T_bd); % 4x4 Matrix
V_b = BodyTwist(V_bmat); % V_b = [w_bi; v_bi]; 6x1 Matrix 
w_b = V_b(1:3); 
v_b = V_b(4:6);

while norm(w_b) > 0.001 || norm(v_b) > 0.0001 && i < 1000
    norm(w_b); norm(v_b);
    th = th + pinv(Jb)*V_b; % 6x1 Matrix (rad)
    th_degree = (th*180/pi) % Angle(degree)

    T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M
    T_bs = inv(T_sb); 

    % New Jacobian // 6x6 Matrix 
    Js1 = s2twist(S1);
    Js2 = ad(expm(S1*th(1)))*s2twist(S2);
    Js3 = ad(expm(S1*th(1))*expm(S2*th(2)))*s2twist(S3);
    Js4 = ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3)))*s2twist(S4);
    Js5 = ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4)))*s2twist(S5);
    Js6 = ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5)))*s2twist(S6);
    Js = [Js1, Js2, Js3, Js4, Js5, Js6];
    Jb = ad(T_bs)*Js; 
    
    % Body Twist 
    T_bd = T_bs*T_sd; % 4x4 Matrix 
    V_bmat = logm(T_bd)*0.1; % 4x4 Matrix
    V_b = BodyTwist(V_bmat); % V_b = [w_b; v_b]; 6x1 Matrix 
    w_b = V_b(1:3); 
    v_b = V_b(4:6);

    i = i+1

end




