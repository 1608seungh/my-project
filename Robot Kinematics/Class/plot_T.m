function plot_T(T, color)

    x_hat = T(1:3,1);
    y_hat = T(1:3,2);
    z_hat = T(1:3,3);
    p = T(1:3,4);

    x_axis = [p, p+x_hat]';
    y_axis = [p, p+y_hat]';
    z_axis = [p, p+z_hat]';            

    plot3(x_axis(:,1), x_axis(:,2), x_axis(:,3), color);
    hold on;
    plot3(y_axis(:,1), y_axis(:,2), y_axis(:,3), color);
    plot3(z_axis(:,1), z_axis(:,2), z_axis(:,3), color);

    axis equal;
    xlim([-4 4])
    ylim([-4 4])
    zlim([-4 4])

    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on;

end