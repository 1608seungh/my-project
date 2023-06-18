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
s6 = servo(a,"D5", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);

% Desired Pos & Orient
x_d =[0.1; -0.15; 0.2];
T_sd = [eye(3), x_d; 0 0 0 1]; 

% Angular Velocity // 3x1 Matrix
w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;-1;0]; w6 = [0;0;1];

% Arbitrary Point on joint axis (q vector)
q1 = [0;0;0]; q2 = [0;0;0]; q3 = [0;0;0.1036];
q4 = [0;0;0.2011]; q5 = [-0.015;-0.081;0.2280]; q6 = [-0.025; -0.081; 0.2280];

% Zero Position
M01 = [eye(3), q2; 0 0 0 1]; % 2nd Joint Position
M02 = [eye(3), q3; 0 0 0 1]; % 3rd Joint Position
M03 = [eye(3), q4; 0 0 0 1]; % 4th Joint Position
M04 = [eye(3), q5; 0 0 0 1]; % 5th Joint Position
M05 = [eye(3), q6; 0 0 0 1]; % 6th Joint Position
M = [eye(3), [-0.015;-0.1516;0.2280]; 0 0 0 1]; 

% Linear Velocity // 3x1 Matrix 
v1 = -cross(w1,q1); v2 = -cross(w2,q2); v3 = -cross(w3,q3); 
v4 = -cross(w4,q4); v5 = -cross(w5,q5); v6 = -cross(w6,q6); 

% Screw Axis // 4x4 Matrix
S1 = ACT.Screw(w1,q1); S2 = ACT.Screw(w2,q2); S3 = ACT.Screw(w3,q3); 
S4 = ACT.Screw(w4,q4); S5 = ACT.Screw(w5,q5); S6 = ACT.Screw(w6,q6);

% Initial Angle 
i = 0;
th = [0; 0; pi/2; -pi/2; 0; 0] % 6x1 Matrix 
% th = [74.3759; -9.1777; 52.5886; -43.4110; 0; -74.3759]; % 원래에서 [0.13;-0.13;0.2]로 움직인 각도
% th = [27.7739; -8.4052; 52.0496; -43.6740; 0.0733; -27.7738]; % [0.13;-0.13;0.2]에서 [0.05;-0.2;0.2]
% th = [34.7433; -42.8668; 47.6136; -4.7469; 0; -34.7433]; % [0.05;-0.2;0.2]에서 [0;-0.1;0.2]
% th = deg2rad(th);

% Arduino Initial Angle
th_deg = rad2deg(th);
ard_th = [th_deg(1); -th_deg(2); th_deg(3); th_deg(4); th_deg(5); 0] + [60; 72; 60; 120; 60; 180]
ard_th_digital = ard_th / 180;

writePosition(s1, ard_th_digital(1));
writePosition(s2, ard_th_digital(2));
writePosition(s3, ard_th_digital(3));
writePosition(s4, ard_th_digital(4));
writePosition(s5, ard_th_digital(5));
writePosition(s6, ard_th_digital(6));

% Tranformation Matrix (Space(fixed) to Body Frame)
T01 = expm(S1*th(1))*M01
T02 = expm(S1*th(1))*expm(S2*th(2))*M02
T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03
T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04
T05 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05
T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M
T_bs = inv(T_sb);

% Jacobian Matrix // 6x6 Matrix
Js1 = ACT.S2twist(S1);
Js2 = ACT.Ad(expm(S1*th(1))) * ACT.S2twist(S2);
Js3 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))) * ACT.S2twist(S3);
Js4 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))) * ACT.S2twist(S4);
Js5 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))) * ACT.S2twist(S5);
Js6 = ACT.Ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))) * ACT.S2twist(S6);
Js = [Js1, Js2, Js3, Js4, Js5, Js6];
Jb = ACT.Ad(T_bs)*Js; % Initial Body Jacobian Matrix / 6x6 Matrix

% Numerical Inverse Kinematic 
T_bd = T_bs*T_sd; % 4x4 Matrix 
V_bmat = logm(T_bd)*0.08; % 4x4 Matrix
V_b = ACT.BodyTwist(V_bmat); % V_b = [w_bi; v_bi]; 6x1 Matrix 
w_b = V_b(1:3); 
v_b = V_b(4:6);

while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && i < 150
    
    dth = pinv(Jb)*V_b;
    th = th + dth; % 6x1 Matrix (rad)

    % Arduino Angle (Trial)
    th_deg = rad2deg(th)'
    ard_th = [th_deg(1); -th_deg(2); th_deg(3); th_deg(4); th_deg(5); 0] + [60; 72; 60; 120; 60; 180]
    ard_th_digital = ard_th / 180;

    writePosition(s1, ard_th_digital(1));
    writePosition(s2, ard_th_digital(2));
    writePosition(s3, ard_th_digital(3));
    writePosition(s4, ard_th_digital(4));
    writePosition(s5, ard_th_digital(5));
    writePosition(s6, ard_th_digital(6));

    i = i+1

    T01 = expm(S1*th(1))*M01;
    T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
    T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
    T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
    T05 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05;
    T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M;
    T_bs = inv(T_sb); 
    disp(T_sb)

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
    w_b = V_b(1:3);
    v_b = V_b(4:6);

    x0 = 0; y0 = 0; z0 = 0;
    x1(i) = T01(1,4); y1(i) = T01(2,4); z1(i) = T01(3,4);
    x2(i) = T02(1,4); y2(i) = T02(2,4); z2(i) = T02(3,4);
    x3(i) = T03(1,4); y3(i) = T03(2,4); z3(i) = T03(3,4);
    x4(i) = T04(1,4); y4(i) = T04(2,4); z4(i) = T04(3,4);
    x5(i) = T05(1,4); y5(i) = T05(2,4); z5(i) = T05(3,4);
    xx(i) = T_sb(1,4); yy(i) = T_sb(2,4); zz(i) = T_sb(3,4);

    p_1 = T01(1:3,4); p_2 = T02(1:3,4); p_3 = T03(1:3,4); 
    p_4 = T04(1:3,4); p_5 = T05(1:3,4); p_6 = T_sb(1:3,4);

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
%     y2_ = y2(i) + radius * cos(angle);
%     z2_ = z2(i) + radius * sin(angle);
%     x2_ = ones(1,length(y2_)) * height/2;
%     point_lid2 = [x2(i)+x2_; y2_; z2_];
%     point_bottom2 = [x2(i)-x2_; y2_; z2_];
%     point2 = [point_lid2, point_bottom2];
%     ones_added = ones(1, length(point2));
% 
%     w2 = [1;0;0];
%     w2_d = T02(1:3,1);
%     a2 = cross(w2, w2_d);
%     a2_norm = norm(a2);
% 
%     if a2_norm == 0
%         w2_hat = w2;
%         theta2 = 0;
%     else
%         theta2 = asin(a2_norm);
%         w2_hat = a2/a2_norm;
%     end
% 
%     R2 = rot.matrix_exp(w2_hat, th(2));
%     T2 = [R2 [x2(i); y2(i); z2(i)]; 0 0 0 1];
%     point2_changed = T2 * [point2; ones_added];
%     plot3(point2(1,:), point2(2,:), point2(3,:)) % 3rd Joint
%     plot3(point2_changed(1,:), point2_changed(2,:), point2_changed(3,:)) % 3rd Joint

    % 4th Joint
    point3_changed = [point1_changed(1,:) + x3(i); point1_changed(2,:) + y3(i); point1_changed(3,:) + z3(i); point1_changed(4,:)];
     plot3(point3_changed(1,:), point3_changed(2,:), point3_changed(3,:)) % 3rd Joint

%     y3_ = y3(i) + radius * cos(angle);
%     z3_ = z3(i) + radius * sin(angle);
%     x3_ = ones(1,length(y3_)) * height/2;
%     point_lid3 = [x3(i)+x3_; y3_; z3_];
%     point_bottom3 = [x3(i)-x3_; y3_; z3_];
%     point3 = [point_lid3, point_bottom3];
%     plot3(point3(1,:), point3(2,:), point3(3,:)) % 4th Joint

    % 5th Joint
    x4_ = x4(i) + radius * cos(angle);
    y4_ = ones(1,length(x4_)) * height/2;
    z4_ = z4(i) + radius * sin(angle);
    point_lid4 = [x4_; y4(i)+y4_; z4_];
    point_bottom4 = [x4_; y4(i)-y4_; z4_];
    point4 = [point_lid4, point_bottom4];
    ones_added = ones(1, length(point4));
    plot3(x4(i), y4(i), z4(i), 'o' ,'color', 'blue', 'Linewidth', 1.5)

%     w5_d = T04(1:3,1);
%     a2 = cross(w2, w2_d);
%     a2_norm = norm(a2);
% 
%     if a2_norm == 0
%         w2_hat = w2;
%         theta2 = 0;
%     else
%         theta2 = asin(a2_norm);
%         w2_hat = a2/a2_norm;
%     end
% 
%     R2 = rot.matrix_exp(w2_hat, th(2));
%     T2 = [R2 [x2(i); y2(i); z2(i)]; 0 0 0 1];
%     point2_changed = T2 * [point2; ones_added];
%     plot3(point2(1,:), point2(2,:), point2(3,:)) % 3rd Joint
%     plot3(point2_changed(1,:), point2_changed(2,:), point2_changed(3,:)) % 3rd Joint



    % plot3(point4(1,:), point4(2,:), point4(3,:)) % 5th Joint

    x5_ = x5(i) + radius * cos(angle);
    y5_ = y5(i) + radius * sin(angle);
    z5_ = ones(1,length(x5_)) * height/2;
    point_lid5 = [x5_; y5_; z5(i)+z5_];
    point_bottom5 = [x5_; y5_; z5(i)-z5_];
    point5 = [point_lid5, point_bottom5];
%     plot3(point5(1,:), point5(2,:), point5(3,:)) % 6th Joint

    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-0.27 0.27])
    ylim([-0.27 0.27])
    zlim([-0.03 0.25])

    line([p_1(1), p_2(1)], [p_1(2), p_2(2)], [p_1(3), p_2(3)])
    line([p_2(1), p_3(1)], [p_2(2), p_3(2)], [p_2(3), p_3(3)])
    line([p_3(1), p_4(1)], [p_3(2), p_4(2)], [p_3(3), p_4(3)])
    line([p_4(1), p_5(1)], [p_4(2), p_5(2)], [p_4(3), p_5(3)])
    line([p_5(1), p_6(1)], [p_5(2), p_6(2)], [p_5(3), p_6(3)], 'color', 'Black', 'Linewidth', 1)
    hold off;

%     long = sqrt((p_6(1)-p_4(1))^2 + (p_6(2)-p_4(2))^2 + (p_6(3)-p_4(3))^2);
%     disp(long)

    pause(0.001)
    
end

% th = [readPosition(s1); readPosition(s2); readPosition(s3); readPosition(s4); readPosition(s5); readPosition(s6)];
% th = th * 180

