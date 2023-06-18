clear all; clc; 

% Desired Position & Orientation of End Effector (원하는 위치값 입력) 
x_d = [0.10; -0.04; 0.26]; 
R_d = [cosd(45) sind(45) 0; -sind(45) cosd(45) 0; 0 0 1];
T_sd = [R_d, x_d; 0 0 0 1]; % 4x4 Matrix

% Zero Position of Robot-Arm (기준이 되는 End-Effector의 현재 위치)
M = [eye(3), [-0.00146;0.0276;0.3709]; 0 0 0 1]; 

% Angular Velocity // 3x1 Matrix
w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;0;1]; w6 = [0;1;0];

% Point on the axis of the Screw (q) // 3x1 Matrix 
q1 = [0;0;0]; q2 = [0;0;0.057]; q3 = [0;0;0.1605];
q4 = [0;0;0.2580]; q5 = [-0.0075;0.0276;0]; q6 = [-0.0121; 0; 0.3651];

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
AdT_bs = ad(T_bs); % 6x6 Matrix 

% Jacobian Matrix // 6x6 Matrix
Js = [w1,w2,w3,w4,w5,w6; v1,v2,v3,v4,v5,v6]; % Initial Space Jacobian Matrix; Jsi = (w_i; v_i)
Jb = AdT_bs*Js; % Initial Body Jacobian Matrix

% Body Twist 
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
    AdT_bs = ad(T_bs);

    % New Jacobian // 6x6 Matrix 
    Js1 = s2twist(S1);
    Js2 = ad(expm(S1*th(1)))*s2twist(S2);
    Js3 = ad(expm(S1*th(1))*expm(S2*th(2)))*s2twist(S3);
    Js4 = ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3)))*s2twist(S4);
    Js5 = ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4)))*s2twist(S5);
    Js6 = ad(expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*expm(S4*th(4))*expm(S5*th(5)))*s2twist(S6);
    Js = [Js1, Js2, Js3, Js4, Js5, Js6];
    Jb = AdT_bs*Js; 
    
    % Body Twist 
    T_bd = T_bs*T_sd; % 4x4 Matrix 
    V_bmat = logm(T_bd)*0.1; % 4x4 Matrix
    V_b = BodyTwist(V_bmat); % V_b = [w_b; v_b]; 6x1 Matrix 
    w_b = V_b(1:3); 
    v_b = V_b(4:6);

    i = i+1

end

t = 0;
dt = 0.1;
for j = 1:100
    t = t + dt;

    % Define joint positions
    p1x = 0;
    p1y = 0;
    p1z = 0.057;

    p2x = 0.1035*sin(th(2))*sin(th(1));
    p2y = 0.1035*sin(th(2))*cos(th(1));
    p2z = 0.1035*cos(th(2));

    p3x = 0.0975*sin(th(2)+th(3))*sin(th(1));
    p3y = 0.0975*sin(th(2)+th(3))*cos(th(1));
    p3z = 0.0975*cos(th(2)+th(3));

    p4x = 0.0276*sin(th(2)+th(3)+pi/2+th(4))*sin(th(1));
    p4y = 0.0276*sin(th(2)+th(3)+pi/2+th(4))*cos(th(1));
    p4z = 0.0276*cos(th(2)+th(3)+pi/2+th(4));

    p5x = 0.1071*sin(th(2)+th(3)+th(4))*sin(th(1))*sin(th(5));
    p5y = 0.1071*sin(th(2)+th(3)+th(4))*cos(th(1))*cos(th(5));
    p5z = 0.1071*cos(th(2)+th(3)+th(4));

Link1 = line([0, p1x], [0, p1y], [0, p1z]);
Link2 = line([p1x, p2x], [p1y, p2y], [p1z, p2z]);
Link3 = line([p2x, p3x], [p2y, p3y], [p2z, p3z]);
Link4 = line([p3x, p4x], [p3y, p4y], [p3z, p4z]);
Link5 = line([p4x, p5x], [p4y, p5y], [p4z, p5z]);

% Define joint positions
p1x = 0;
p1y = 0;
p1z = 0.057;

p2x = 0.1035*sin(th(2))*sin(th(1));
p2y = 0.1035*sin(th(2))*cos(th(1));
p2z = 0.1035*cos(th(2));

p3x = 0.0975*sin(th(2)+th(3))*sin(th(1));
p3y = 0.0975*sin(th(2)+th(3))*cos(th(1));
p3z = 0.0975*cos(th(2)+th(3));

p4x = 0.0276*sin(th(2)+th(3)+pi/2+th(4))*sin(th(1));
p4y = 0.0276*sin(th(2)+th(3)+pi/2+th(4))*cos(th(1));
p4z = 0.0276*cos(th(2)+th(3)+pi/2+th(4));

p5x = 0.1071*sin(th(2)+th(3)+th(4))*sin(th(1))*sin(th(5));
p5y = 0.1071*sin(th(2)+th(3)+th(4))*cos(th(1))*cos(th(5));
p5z = 0.1071*cos(th(2)+th(3)+th(4));

Link1 = line([0, p1x], [0, p1y], [0, p1z]);
Link2 = line([p1x, p2x], [p1y, p2y], [p1z, p2z]);
Link3 = line([p2x, p3x], [p2y, p3y], [p2z, p3z]);
Link4 = line([p3x, p4x], [p3y, p4y], [p3z, p4z]);
Link5 = line([p4x, p5x], [p4y, p5y], [p4z, p5z]);

% Plot robot arm
figure;
hold on;
plot3([x1 x2],[y1 y2],[z1 z2],'LineWidth',3);
plot3([x2 x3],[y2 y3],[z2 z3],'LineWidth',3);
plot3([x3 x4],[y3 y4],[z3 z4],'LineWidth',3);
plot3([x4 x5],[y4 y5],[z4 z5],'LineWidth',3);
plot3([x5 x6],[y5 y6],[z5 z6],'LineWidth',3);

plot3(x1,y1,z1,'bo','MarkerSize',10,'LineWidth',3);
plot3(x2,y2,z2,'ro','MarkerSize',10,'LineWidth',3);
plot3(x3,y3,z3,'go','MarkerSize',10,'LineWidth',3);
plot3(x4,y4,z4,'yo','MarkerSize',10,'LineWidth',3);
plot3(x5,y5,z5,'bo','MarkerSize',10,'LineWidth',3);
plot3(x6,y6,z6,'ro','MarkerSize',10,'LineWidth',3);

axis equal;
axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('6 DOF Robot Arm Link Visualization');
legend('Link 1','Link 2','Link 3', 'Link 4', 'Link 5', ...
       'Joint 1','Joint 2','Joint 3','Joint 4', 'Joint 5','End Effector');
