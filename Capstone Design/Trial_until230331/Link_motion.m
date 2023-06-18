theta1 = [ 0 5 10 15 20 25 30 35 40 45 50 55 60 55 50 45 40 35 30 35 40 45 50 55 60 65 75 65 60 55 50 45 40 35 30];
size_theta = size(theta1);
t_f = size_theta(1,2);

for t = 1:1:t_f
    Link1_a = [0 0];
    Link1_b = [3 0];

    RotMat1 = TwoD_RMat(theta1(t));

    Link1_a = TwoD_RTP(RotMat1, Link1_a);
    Link1_b = TwoD_RTP(RotMat1, Link1_b);

    Global_Origin = [0 0];

    Global_x = [1 0];
    Global_y = [0 1];

    plot([Global_Origin(1) Global_x(1)], [Global_Origin(2) Global_x(2)], '-r');
    hold on

    plot([Global_Origin(1) Global_y(1)], [Global_Origin(2) Global_y(2)], '-g');
    hold on

    plot([Link1_a(1) Link1_b(1)], [Link1_a(2) Link1_b(2)], '-k', 'Marker', 'o', 'LineWidth', 2, ...
        'MarkerSize', 10, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', [0.5 0.5 0.5]);
    hold on

    axis([-2 5 -2 5]);
    daspect([1,1,1]);
    pause(0.2);

    hold off
end

