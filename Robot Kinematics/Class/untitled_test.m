% fwd_kin_test

robot = serial_fwd_kin;
z_hat = [0;0;1];
y_hat = [0;1;0];
x_hat = [1;0;0];

w = [z_hat, x_hat, x_hat, x_hat, y_hat, z_hat];
q = [[0;0;0], [0;0;0.029], [0;0.0832;0.029], [0;0.1664;0.029], [0;0.01664;0.029], [0;0.2395;0.029]]; % M을 바꿔주어야 함...
robot.M_end = [eye(3) [0;3126;0.029]; 0 0 0 1];

robot.w = w;
robot.q = q;



for t = 0:0.1:5

    theta = [0; 0.1*t; 0.15*t; 1-0.02*t^2; 0; 0];

    robot.theta = theta;
    robot = robot.calculate;
    robot = robot.draw_linkage;


    pause(0.01)
    clf

end