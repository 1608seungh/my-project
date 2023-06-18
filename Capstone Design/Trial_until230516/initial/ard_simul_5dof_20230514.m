clear all; clc; 
clear a; % Arduino Clear

ACT = Func20230403;
rot = rotations;
tra = transformations;

% Arduino Setting
a = arduino();
s1 = servo(a,"D2", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s2 = servo(a,"D3", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s3 = servo(a,"D9", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s4 = servo(a,"D8", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s5 = servo(a,"D4", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);

% Desired Pos & Orient
x_d =[0.15; 0.07; 0.2]; % Current Position [-0.15; 0.03; 0.2]
T_sd = [eye(3), x_d; 0 0 0 1]; 

% Angular Velocity // 3x1 Matrix
w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;1;0];

% Arbitrary Point on joint axis (q vector)
q1 = [0;0;0]; q2 = [0;0;0]; q3 = [0;0;0.1036];
q4 = [0;0;0.2011]; q5 = [0; 0.081; 0.2011]; % 0.2280

% Zero Position
M01 = [eye(3), q2; 0 0 0 1]; % 2nd Joint Position
M02 = [eye(3), q3; 0 0 0 1]; % 3rd Joint Position
M03 = [eye(3), q4; 0 0 0 1]; % 4th Joint Position
M04 = [eye(3), q5; 0 0 0 1]; % 5th Joint Position
M = [eye(3), [0; 0.1516; 0.2011]; 0 0 0 1]; 

% Linear Velocity // 3x1 Matrix 
v1 = -cross(w1,q1); v2 = -cross(w2,q2); v3 = -cross(w3,q3); 
v4 = -cross(w4,q4); v5 = -cross(w5,q5);  

% Screw Axis // 4x4 Matrix
S1 = ACT.Screw(w1,q1); S2 = ACT.Screw(w2,q2); S3 = ACT.Screw(w3,q3); 
S4 = ACT.Screw(w4,q4); S5 = ACT.Screw(w5,q5); 

% Initial Angle 
i = 0;
% th = [pi/4; pi/9; -pi/6; pi/3; 0] % 6x1 Matrix 
th = [1.37396987116304;	0.471289049839597;	-1.02607569476437;	0.724885219843945;	2.78102914010252e-17];
% th = [74.3759; -9.1777; 52.5886; -43.4110; 0; -74.3759]; % 원래에서 [0.13;-0.13;0.2]로 움직인 각도
% th = [27.7739; -8.4052; 52.0496; -43.6740; 0.0733; -27.7738]; % [0.13;-0.13;0.2]에서 [0.05;-0.2;0.2]
% th = [34.7433; -42.8668; 47.6136; -4.7469; 0; -34.7433]; % [0.05;-0.2;0.2]에서 [0;-0.1;0.2]
% th = deg2rad(th);

% Arduino Initial Angle
th_deg = rad2deg(th);
ard_th = [th_deg(1); th_deg(2); -th_deg(3); -th_deg(4); th_deg(5)] + [90; 90; 90; 180; 90]
ard_th_digital = ard_th / 180;

writePosition(s1, ard_th_digital(1));
writePosition(s2, ard_th_digital(2));
writePosition(s3, ard_th_digital(3));
writePosition(s4, ard_th_digital(4));
writePosition(s5, ard_th_digital(5));

% Tranformation Matrix (Space(fixed) to Body Frame)
T01 = expm(S1*th(1))*M01
T02 = expm(S1*th(1))*expm(S2*th(2))*M02
T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03
T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04
T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M
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
w_b = V_b(1:3); v_b = V_b(4:6);

while norm(v_b) > 0.0001 && i < 200
    
%     dth = pinv(Jb)*V_b;
    Jb_ = Jb(4:6,:);
    gamma = 0.1;
    th_home = [pi/4; pi/9; -pi/4; pi/3; 0];
    dth = pinv(Jb_)*V_b(4:6) + gamma*(eye(5)-pinv(Jb_)*Jb_)*(th_home-th);
    th = th + dth; % 6x1 Matrix (rad)

    % Arduino Angle (Trial)
    th_deg = rad2deg(th)'
    ard_th = [th_deg(1); th_deg(2); -th_deg(3); -th_deg(4); th_deg(5)] + [90; 90; 90; 180; 90]
    ard_th_digital = ard_th / 180;

    writePosition(s1, ard_th_digital(1));
    writePosition(s2, ard_th_digital(2));
    writePosition(s3, ard_th_digital(3));
    writePosition(s4, ard_th_digital(4));
    writePosition(s5, ard_th_digital(5));

    i = i+1

    T01 = expm(S1*th(1))*M01;
    T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
    T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
    T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
    T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M;
    T_bs = inv(T_sb); 
    disp(T_sb)

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
    xx(i) = T_sb(1,4); yy(i) = T_sb(2,4); zz(i) = T_sb(3,4);

    p_1 = T01(1:3,4); p_2 = T02(1:3,4); p_3 = T03(1:3,4); 
    p_4 = T04(1:3,4); p_5 = T_sb(1:3,4);

    clf;
    plot3(x0,y0,z0,'x', 'color', 'black', 'Linewidth', 1)
    hold on;
    plot3(xx,yy,zz,'color', 'red', 'Linewidth', 0.5)

    radius = 0.007;
    height = 0.03;
    angle = 0:2*pi/100:2*pi;

    % 1st Joint
    x0_ = x0 + radius * cos(angle);
    y0_ = y0 + radius * sin(angle);
    z0_ = ones(1,length(x0_)) * height/2;
    point_lid0 = [x0_; y0_; z0+z0_];
    point_bottom0 = [x0_; y0_; z0-z0_];
    point0 = [point_lid0, point_bottom0];
    plot3(point0(1,:), point0(2,:), point0(3,:))

    %2nd Joint
    y1_ = y1(i) + radius * cos(angle);
    z1_ = z1(i) + radius * sin(angle);
    x1_ = ones(1,length(y1_)) * height/2;
    point_lid1 = [x1(i)+x1_; y1_; z1_];
    point_bottom1 = [x1(i)-x1_; y1_; z1_];
    point1 = [point_lid1, point_bottom1];
    ones_added = ones(1, length(point1));
       
    w1_d = T01(1:3,3);
    a1 = cross(w1, w1_d);
    a1_norm = norm(a1);

    if a1_norm == 0
        w1_hat = w1;
        theta1 = 0;
    else
        theta1 = asin(a1_norm);
        w1_hat = a1/a1_norm;
    end

    R1 = rot.matrix_exp(w1_hat, th(1));
    T1 = [R1 [x1(i); y1(i); z1(i)]; 0 0 0 1];
    point1_changed = T1 * [point1; ones_added];
    plot3(point1_changed(1,:), point1_changed(2,:), point1_changed(3,:))

    % 3rd Joint
    point2_changed = [point1_changed(1,:) + x2(i); point1_changed(2,:) + y2(i); point1_changed(3,:) + z2(i); point1_changed(4,:)];
    plot3(point2_changed(1,:), point2_changed(2,:), point2_changed(3,:)) % 3rd Joint

    % 4th Joint
    point3_changed = [point1_changed(1,:) + x3(i); point1_changed(2,:) + y3(i); point1_changed(3,:) + z3(i); point1_changed(4,:)];
     plot3(point3_changed(1,:), point3_changed(2,:), point3_changed(3,:)) % 3rd Joint

    % 5th Joint
    plot3(x4(i), y4(i), z4(i), 'o' ,'color', 'blue', 'Linewidth', 1.5)

    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-0.27 0.27])
    ylim([-0.27 0.27])
    zlim([-0.03 0.25])

    line([p_1(1), p_2(1)], [p_1(2), p_2(2)], [p_1(3), p_2(3)])
    line([p_2(1), p_3(1)], [p_2(2), p_3(2)], [p_2(3), p_3(3)])
    line([p_3(1), p_4(1)], [p_3(2), p_4(2)], [p_3(3), p_4(3)])
    line([p_4(1), p_5(1)], [p_4(2), p_5(2)], [p_4(3), p_5(3)])
    hold off;

    pause(0.001)
    
end