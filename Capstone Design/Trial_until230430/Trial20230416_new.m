clear all; clc; clf;

S1 = [0 -1 0 0; 1 0 0 0; 0 0 0 0; 0 0 0 0];
S2 = [0 0 -1 0; 0 0 0 0; 1 0 0 -10; 0 0 0 0];
S3 = [0 0 0 0; 0 0 -1 10; 0 1 0 0; 0 0 0 0];

M01 = eye(4);
M02 = [0 1 0 10; 0 0 -1 0; -1 0 0 0; 0 0 0 1];
M03 = [0 0 1 10; 0 1 0 0; -1 0 0 -10; 0 0 0 1];

th1 = 0; th2 = pi/6; th3 = pi/6;
th = [th1; th2; th3];
T01 = expm(S1*th(1))*M01;
T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;

x2 = T02(1,4); y2 = T02(2,4); z2 = T02(3,4);
P_2 = [x2 y2 z2];
xx = T03(1,4); yy = T03(2,4); zz = T03(3,4);
P_e = [xx yy zz];
x = 0; y = 0; z = 0;
p_1 = T01(1:3,4); p_2 = T02(1:3,4); p_3 = T03(1:3,4);

t = 1:10;
dt = 0.1;

for i = 1:length(t)
    th(1) = pi/6+pi/10*i; th(2) = -pi/14*i+pi/70*i^2;

    T01 = expm(S1*th(1))*M01;
    T02 = expm(S1*th(1))*expm(S2*th(2))*M02;
    T03 = expm(S1*th(1))*expm(S2*th(2))*expm(S3*th(3))*M03;

    x2(i) = T02(1,4); y2(i) = T02(2,4); z2(i) = T02(3,4);
    xx(i) = T03(1,4); yy(i) = T03(2,4); zz(i) = T03(3,4);
    P_e = [xx(i) yy(i) zz(i)];
    p_1 = T01(1:3,4); p_2 = T02(1:3,4); p_3 = T03(1:3,4);

    i = i+1;

    plot3(x,y,z,'o', 'color', 'Black', 'LineWidth', 3)
    hold on
    plot3(x2,y2,z2, 'color', 'Blue', 'LineWidth', 0.2) 
    plot3(xx,yy,zz, 'color', 'Black', 'LineWidth', 0.2)
    hold off

    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-20 20])
    ylim([-20 20])
    zlim([-20 20])
    grid on

    line([p_1(1), p_2(1)], [p_1(2), p_2(2)], [p_1(3), p_2(3)], 'Color','red', 'LineWidth', 2)
    line([p_2(1), p_3(1)], [p_2(2), p_3(2)], [p_2(3), p_3(3)], 'Color','Green', 'LineWidth', 2)
    view(3) % Line을 3차원으로 보기

end
