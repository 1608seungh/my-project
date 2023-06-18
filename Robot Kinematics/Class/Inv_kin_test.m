% Inv_kin_test

robot1 = serial_fwd_kin;
z_hat = [0;0;1];
y_hat = [0;1;0];
x_hat = [1;0;0];

w = [z_hat, z_hat, z_hat, z_hat, z_hat, z_hat, z_hat];
q = [[0;0;1], [0;1;1], [0;2;1], [0;3;1], [0;4;1], [0;5;1], [0;6;1]];
robot1.M_end = [eye(3) [0;7;1]; 0 0 0 1];

robot1.w = w;
robot1.q = q;

abc = rotations;
R_des = abc.RotZ(pi/2);
x_d = [3; 4; 1];

T_des = [R_des, x_d; 0 0 0 1];
robot1.draw_frame(T_des)

theta = zeros(7,1);
dt = 0.05;
epsilon = 0.08;
twist_body = 100;

figure(1)


while norm(twist_body) > epsilon
    clf
    
    % Inverse Kinematic
    robot1.theta = theta;
    robot1 = robot1.calculate;
    T_sb = robot1.M_(:,:,end);

    s_theta = logm(inv(T_sb)*T_des);
    w_theta_vec = [-s_theta(2,3); s_theta(1,3); -s_theta(1,2)];
    v_vec = s_theta(1:3, 4);
    twist_body = [w_theta_vec; v_vec];
    
    % Null Space Optimization
    J_pseudo = pinv(robot1.Jb);
    theta_center = zeros(7,1);
    theta_center(2) = -pi/2;
    grad_H = zeros(7,1);
    grad_H(2) = theta(2) - theta_center(2);
    gamma = -1;
    dth = J_pseudo*twist_body + gamma*(eye(7) - J_pseudo*robot1.Jb)*grad_H;
    theta = theta + dth * dt;


    % orientation
%     Rsb = T_sb(1:3,1:3);
%     Rb_des = inv(Rsb)*R_des;
%     w_theta = logm(Rb_des);
%     w_theta_vec = [-w_theta(2,3); w_theta(1,3); -w_theta(1,2)];
%     Jbw = robot1.Jb(1:3, :);
%     dth = pinv(Jbw) * w_theta_vec;

    % position
%     x = T_sb(1:3, 4);   
%     diff = norm(x_d - x);
    
%     x_db = inv(T_sb) * [x_d; 1];
%     x_db = x_db(1:3);

%     v_b = x_db/norm(x_db);
%     Jbv = robot1.Jb(4:6, :);
%     dth = inv(Jbv) * v_b;
%     dth = pinv(Jbv) * v_b;
    theta = theta + dth * dt;

    % Plotting
    subplot(2,2,1)
    robot1 = robot1.draw_linkage;
    robot1.draw_frame(T_des)
    view([1,0,0])
    
    subplot(2,2,2)
    robot1 = robot1.draw_linkage;
    robot1.draw_frame(T_des)
    view([0,1,0])

    subplot(2,2,3)
    robot1 = robot1.draw_linkage;
    robot1.draw_frame(T_des)
    view([0,0,1])

    subplot(2,2,4)
    robot1 = robot1.draw_linkage;
    robot1.draw_frame(T_des)

    pause(0.01)
  

end