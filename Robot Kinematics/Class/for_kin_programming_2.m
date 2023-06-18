% fwd_kin_test

robot1 = serial_fwd_kin;
z_hat = [0;0;1];
y_hat = [0;1;0];
x_hat = [1;0;0];

w = [z_hat, y_hat, x_hat];
q = [[0;0;1], [0;1;1], [0;2;1]];
robot1.M_end = [eye(3) [0;3;1]; 0 0 0 1];

robot1.w = w;
robot1.q = q;

figure(1)

for t = 0:0.1:1
    clf;
%     theta = [0.2*t; -0.4*t; 0; -0.4*t];
    theta = [0; 0; 0];

    robot1.theta = theta;
    robot1 = robot1.calculate;

    subplot(2,2,1)
    robot1 = robot1.draw_linkage;
    view([1,0,0])
    
    subplot(2,2,2)
    robot1 = robot1.draw_linkage;
    view([0,1,0])

    subplot(2,2,3)
    robot1 = robot1.draw_linkage;
    view([0,0,1])

    subplot(2,2,4)
    robot1 = robot1.draw_linkage;

    pause(0.01)

end