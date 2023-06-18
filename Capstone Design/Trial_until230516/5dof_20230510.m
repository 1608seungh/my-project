clear all; clc; 
clear a; % Arduino Clear

ACT = Func20230403;
rot = rotations;
tra = transformations;

% Desired Position & Orientation of End Effector 
x_d = [0.15; 0.05; 0.1];
R_d = [1 0 0; 0 1 0; 0 0 1];
T_sd = [R_d, x_d; 0 0 0 1]; % 4x4 Matrix

% Zero Position
M01 = [eye(3), [0; 0; 0]; 0 0 0 1]; % 1st & 2nd Joint Position (1,2번 회전축이 만나는 지점을 원점으로 설정했으므로)
M02 = [eye(3), [0; 0.1036; 0]; 0 0 0 1]; % 3rd Joint Position
M03 = [eye(3), [0; 0.2012; 0]; 0 0 0 1]; % 4th Joint Position
M04 = [eye(3), [0.0097; 0.2012; 0.0309]; 0 0 0 1]; % 5th Joint Position
M05 = [eye(3), [0.0229; 0.2833; 0.0309]; 0 0 0 1]; % 6th Joint Position
M = [eye(3), [0.0097; 0.3583; 0.0309]; 0 0 0 1]; 

% Angular Velocity // 3x1 Matrix
w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;1;0]; w6 = [0;0;1];

% Point on the axis of the Screw (q) // 3x1 Matrix 
q1 = [0; 0; 0]; q2 = [0; 0; 0]; q3 = [0; 0.1036; 0];
q4 = [0; 0.2012; 0]; q5 = [0.0097; 0; 0.0309]; q6 = [0.0229; 0.2833; 0];

% Linear Velocity // 3x1 Matrix 
v1 = -cross(w1,q1); v2 = -cross(w2,q2); v3 = -cross(w3,q3); 
v4 = -cross(w4,q4); v5 = -cross(w5,q5); v6 = -cross(w6,q6); 

% Screw Axis // 4x4 Matrix // [w_mat, v; 0 0 0 0]
S1 = ACT.Screw(w1,q1); S2 = ACT.Screw(w2,q2); S3 = ACT.Screw(w3,q3); 
S4 = ACT.Screw(w4,q4); S5 = ACT.Screw(w5,q5); S6 = ACT.Screw(w6,q6);

% Initial Angle 
i = 0;
% th = [74.3793; -17.1952; 73.4964; -56.3012; 0; -74.3793];
% th = deg2rad(th)
th = [0; pi/2; -pi/2; 0; 0; 0]; % 6x1 Matrix 

% Arduino Initial Angle
th_deg = rad2deg(th);
ard_th = [th_deg(1); -th_deg(2); th_deg(3); -th_deg(4); th_deg(5); th_deg(6)] + [60; 72; 60; 30; 60; 90];
ard_th_digital = ard_th / 180';

% Tranformation Matrix (Space(fixed) to Body Frame)
T01 = expm(S1*th(1))*M01;
T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;
T04 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*M04;
T05 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*M05
T_sb = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5))*expm(S6*th(6))*M
T_bs = inv(T_sb);

% Jacobian Matrix // 6x6 Matrix
Js = [w1,w2,w3,w4,w5,w6; v1,v2,v3,v4,v5,v6]; % Initial Space Jacobian Matrix; Jsi = (w_i; v_i)
Jb = ACT.Ad(T_bs)*Js; % Initial Body Jacobian Matrix / 6x6 Matrix

% Numerical Inverse Kinematic 
T_bd = T_bs*T_sd; % 4x4 Matrix 
V_bmat = logm(T_bd)*0.07; % 4x4 Matrix
V_b = ACT.BodyTwist(V_bmat); % V_b = [w_bi; v_bi]; 6x1 Matrix 
w_b = V_b(1:3); 
v_b = V_b(4:6);

while (norm(w_b) > 0.001 || norm(v_b) > 0.0001) && i < 1000
    
    dth = pinv(Jb)*V_b;
    th = th + dth; % 6x1 Matrix (rad)

    % Arduino Angle (Trial)
    th_deg = rad2deg(th)'
    ard_th = [th_deg(1); -th_deg(2); th_deg(3); -th_deg(4); th_deg(5); th_deg(6)] + [60; 72; 60; 30; 60; 90];
    ard_th_digital = ard_th / 180';

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
    disp(Js)
    
    T_bd = T_bs*T_sd; % 4x4 Matrix 
    V_bmat = logm(T_bd)*0.07; % 4x4 Matrix
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

    radius = 0.007; height = 0.03; angle = 0:2*pi/100:2*pi;

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

    w4_d = T04(1:3,1);
    a4 = cross(w4, w4_d);
    a4_norm = norm(a4);

    if a4_norm == 0
        w4_hat = w4
        theta5 = 0;
    else
        theta5 = asin(a4_norm);
        w4_hat = a4/a4_norm;
    end

    R4 = rot.matrix_exp(w4_hat, th(4));
    T4 = [R4 [x4(i); y4(i); z4(i)]; 0 0 0 1];
    point4_changed = T4 * [point4; ones_added];
    plot3(point4(1,:), point4(2,:), point4(3,:)) % 3rd Joint
    plot3(point4_changed(1,:), point4_changed(2,:), point4_changed(3,:)) % 5rd Joint


%     plot3(point4(1,:), point4(2,:), point4(3,:)) % 5th Joint

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
    line([p_4(1), p_5(1)], [p_4(2), p_5(2)], [p_4(3), p_5(3)], 'color', 'black', 'linewidth', 1)
    line([p_5(1), p_6(1)], [p_5(2), p_6(2)], [p_5(3), p_6(3)], 'color', 'Black', 'Linewidth', 2)
    hold off;

    pause(0.0001)
    
end