% Prismatic Joint with IK calculation 0605

robot1 = serial_func_0605;
z_hat = [0;0;1]; y_hat = [0;1;0]; x_hat = [1;0;0];
zero_hat = zeros(3,1);

robot1.M_end = [eye(3) [0;3;1]; 0 0 0 1];
robot1.w = [z_hat, [0;0;0], x_hat]; % Prismatic Joint : w = [0;0;0];
robot1.q = [[0;0;1], [0;1;1], [0;2;1]];
robot1.v = [zero_hat, y_hat, zero_hat]; % Dermi
[~, n_joint] = size(robot1.w); % number of columns

abc = rotations;
R_des = abc.RotZ(pi/2);
x_d = [-1; 4; 2];

T_des = [R_des, x_d; 0 0 0 1];

theta = zeros(n_joint, 1);
dt = 0.1;
epsilon = 0.01;
twist_body = ones(6,1)*100;

figure(1)


while norm(twist_body(4:6)) > epsilon
    clf
    
    % Inverse Kinematic
    robot1.theta = theta;
    robot1 = robot1.calculate;
    T_sb = robot1.M_(:,:,end);

    s_theta = logm(inv(T_sb)*T_des);
    w_theta_vec = [-s_theta(2,3); s_theta(1,3); -s_theta(1,2)];
    v_vec = s_theta(1:3, 4);
    twist_body = [w_theta_vec; v_vec]; % position만 조절 
    
%     % Null Space Optimization
%     J_pseudo = pinv(robot1.Jb);
%     theta_center = zeros(3,1);
%     theta_center(2) = -pi/2;
%     grad_H = zeros(7,1);
%     grad_H(2) = theta(2) - theta_center(2);
%     gamma = -1;
%     dth = J_pseudo*twist_body + gamma*(eye(7) - J_pseudo*robot1.Jb)*grad_H;
%     theta = theta + dth * dt;

    J_bv = robot1.Jb(4:6, :);
    dth = inv(J_bv) * twist_body(4:6);
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