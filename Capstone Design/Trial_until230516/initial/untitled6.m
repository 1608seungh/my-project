clear all; clc; 

Fun = Func20230515;
mat = mathmatics0515;

rot = rotations;
tra = transformations;

% Desired Pos & Orient
x_d = [0.18; 0.08; 0.2];
T_sd = [eye(3), x_d; 0 0 0 1]; 
th = [-pi/4; -pi/6; -pi/12; pi/6; 0]; % Initial Angle

% Need to Calculate
w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0]];
q = [[0;0;0], [0;0;0], [0;0;0.1036], [0;0;0.2011], [0.015; 0.081; 0.2280]];
M_end = [eye(3), [0.015; 0.1516; 0.2280]; 0 0 0 1];

v = zeros(3, length(w));
S = zeros(6, length(w));
M = zeros(4, 4, length(w)+1);
M(:, :, end) = [eye(3), [0.015; 0.1516; 0.2280]; 0 0 0 1];
exp = zeros(4, 4, length(w));

k = 0;
v_b = [1;0;0];
while norm(v_b) > 0.00005 && k < 200

    i = 0;
    for q_idx = q
        i = i+1;
        v(:, i) = -cross(w(:, i), q_idx);
        S(:, i) = [w(:, i); v(:, i)];
        M(:, :, i) = [eye(3), q_idx; 0 0 0 1];
        exp(:, :, i) = tra.screw_exp(S(:, i), th(i));
    end
    
    Trs(:, :, 1) = M(:, :, 1);
    for i = 1:length(w)
    
        Trs(:, :, i+1) = M(:, :, i+1);
    
        for j = i:-1:1
            Trs(:, :, i+1) = exp(:, :, j) * Trs(:, :, i+1);
        end
    end
    
    Js = zeros(6, length(w));
    Js(:, 1) = S(:, 1);
    for i = 2:length(w)
        T = eye(4);
        for j = 1:i-1
            T = T*exp(:, :, j);
        end
        Js(:,i) = tra.Ad(T) * S(:,i);
    end
    
    T_sb = Trs(:, :, end);
    T_bs = inv(T_sb);
    Jb = tra.Ad(T_bs)*Js;

    T_bd = T_bs*T_sd;
    V_bmat = logm(T_bd)*0.07;
    V_b = mat.BodyTwist(V_bmat);
    v_b = V_b(4:6);

    Jb_ = Jb(4:6,:);
    gamma = 0.2;
    th_home = [-pi/4; -pi/6; -pi/12; pi/6; 0];
    dth = pinv(Jb_)*V_b(4:6) + gamma*(eye(5)-pinv(Jb_)*Jb_)*(th_home-th);
    th = th + dth; % 6x1 Matrix (rad)
    th_deg = rad2deg(th)'

    k = k+1

    xx(k) = Trs(1, 4, 6);  yy(k) = Trs(2, 4, 6);  zz(k) = Trs(3, 4, 6); 
    P_k = [xx(k), yy(k), zz(k)]
    pos_err = abs(P_k-x_d') / x_d' * 100 % Relative Error

    plot3(xx,yy,zz,'color', 'red', 'Linewidth', 0.5)

end

grid on;
xlabel('x'); ylabel('y'); zlabel('z');
xlim([-0.3 0.3])
ylim([-0.3 0.3])
zlim([-0.02 0.3])



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
    

