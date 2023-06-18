clear all; clc;


w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;0;1];
q1 = [0;0;0]; q2 = [0; 0; 0.073]; q3 = [0; 0.083; 0.073]; q4 = [0; 0.166; 0.073]; q5 = [0; 0.166; 0.033];
v1 = -cross(w1,q1); v2 = -cross(w2,q2); v3 = -cross(w3,q3); v4 = -cross(w4,q4); v5 = -cross(w5,q5); 
M = [eye(3),[0; 0.166; -0.01]; 0, 0, 0, 1];

S1 = screw(w1,q1);
S2 = screw(w2,q2);
S3 = screw(w3,q3);
S4 = screw(w4,q4);
S5 = screw(w5,q5);

x_d = [0.04; 0.14; 0.07];
T_sd = [eye(3), x_d; 0 0 0 1];

% th = [-86.9474; 41.3064; -44.3338; 3.0274; 86.9474]; % Initial Angle
th = [-81.2715; 53.2575; -43.9620; -9.2955; 81.2715];
th = deg2rad(th);
w_b = [1;1;1]; v_b = [1;1;1];

k = 1; 
while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && k < 200

    T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M
    T_bs = inv(T_sb);
    
    AdT_bs = Ad(T_bs);
    
    Js1 = [w1; v1];
    Js2 = Ad(expm(S1*th(1)))*[w2;v2];
    Js3 = Ad(expm(S1*th(1))*expm(S2*th(2)))*[w3;v3];
    Js4 = Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3)))*[w4;v4];
    Js5 = Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4)))*[w5;v5];
    Js = [Js1, Js2, Js3, Js4, Js5];
    Jb = AdT_bs * Js;
    
    T_bd = T_bs*T_sd;
    V_bmat = logm(T_bd)*0.1;
    w_b = [V_bmat(3,2); V_bmat(1,3); V_bmat(2,1)];
    v_b = V_bmat(1:3,4);
    V_b = [w_b; v_b];
    
    th_center = zeros(5,1);
    th_center(3) = -pi/6;
    th_center(4) = pi/6;
    grad_H = zeros(5,1);
    grad_H(3) = th(3) - th_center(3);
    grad_H(4) = th(4) - th_center(4);

    gamma = -0.2;
    dth = pinv(Jb)*V_b + gamma*(eye(5)-pinv(Jb)*Jb) * grad_H;
    
    th = th + dth;
    th_deg = rad2deg(th)

    k = k+1;

end




