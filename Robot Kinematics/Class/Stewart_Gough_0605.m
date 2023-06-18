% Stewart_Gough_Platform with IK calculation 0605
clear all; clc; clf;

rot = rotations;
x_hat = [1;0;0]; y_hat = [0;1;0]; z_hat = [0;0;1]; zero_hat = zeros(3,1);
chain = cell(6,1);
theta = cell(6,1);

% Definition of Serial Chains
for i = 1:6
    R = rot.RotZ((i-1)*pi/3); % 60 degree rotation
    chain{i} = serial_func_0605;
    chain{i}.M_end = [eye(3) [0;0;sqrt(3)]; 0 0 0 1]; 
    chain{i}.w = R * [x_hat, y_hat, z_hat, zero_hat, x_hat, y_hat]; 
    chain{i}.q = R * [[0;2;0], [0;2;0], [0;2;0], [0;2;0], [0;1;sqrt(3)], [0;1;sqrt(3)]];
    chain{i}.v = R * [zero_hat, zero_hat, zero_hat, [-1/2; sqrt(3)/2; 0], zero_hat, zero_hat]; % Dermi
    [~, n_joint] = size(chain{i}.w); % number of columns
    theta{i} = zeros(n_joint, 1);

end

% Target of End-Effector Position
tra = transformations;
% R_des = rot.RotZ(pi/8);
R_des = rot.RotX(pi/8); % All direction possible!! 
x_d = [0; 0; sqrt(3)+0.2];
T_des = [R_des, x_d; 0 0 0 1];

% Other Parameters 
dt = 0.1;
epsilon = 0.01;
twist_body = ones(6,1)*100;

figure(1)

while norm(twist_body(4:6)) > epsilon
    clf;
    
    for j = 1:6
        chain{j}.theta = theta{j};
        chain{j} = chain{j}.calculate;
    end

    T_sb = chain{1}.M_(:,:,end); % 1,2,3,4,5,6 Everything Possible 

    % Body Twist 
    s_theta = logm(inv(T_sb)*T_des); 
    w_theta_vec = [-s_theta(2,3); s_theta(1,3); -s_theta(1,2)];
    v_vec = s_theta(1:3, 4);
    twist_body = [w_theta_vec; v_vec]; 

    % Calculating Jd
    Jd = zeros(30,36);

    for i = 1:5
        Jd(6*i-5: 6*i, 6*i-5: 6*i+6) = [chain{i}.Jb,  -chain{i+1}.Jb];
    end

    Ha = Jd(:, [4 10 16 22 28 34]); % 30x6 matrix
    Jd(:, [4 10 16 22 28 34]) = []; % Remove 'Ha componenets' of Jd
    Hp = Jd; % 30x30 matrix
    Jc = -pinv(Hp) * Ha;
    Ja = chain{1}.Jb * [Jc(1:3, :); [1 0 0 0 0 0]; Jc(4:5, :)];
    dth_a = pinv(Ja) * twist_body; % Active Joint Angle

    if norm(dth_a) > 1
        dth_a = dth_a / norm(dth_a);
    end

    dth_p = Jc * dth_a; % Passive Joint Angle

    % Distributing each Joint angle
    for i = 1:6
        dth = [dth_p(5*i-4: 5*i-2); dth_a(i); dth_p(5*i-1: 5*i)]; % [S Joint, P joint, U joint] for each Chian
        theta{i} = theta{i} + dth * dt;
    end

    % Plotting
    for i = 1:6
        chain{i} = chain{i}.draw_linkage;
        chain{i}.draw_frame(T_des)
    end

    drawnow;
  
end