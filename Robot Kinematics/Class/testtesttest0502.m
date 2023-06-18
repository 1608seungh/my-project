% test
clear all; clc;

radius = 0.001
height = 0.006

rot = rotations;

w1 = [0;0;1]; th = pi/4;
x1 = 0.02 ; y1 = 0.01; z1 = 0.03;

angle = 0:2*pi/100:2*pi;
x = x1 + radius * cos(angle);
y = y1 + radius * sin(angle);
z = ones(1,length(x)) * height/2;
point_lid = [x;y;z1+z];
point_bottom = [x;y;z1-z];
point1 = [point_lid, point_bottom];
ones_added = ones(1, length(point1));

xlabel('x');
ylabel('y');
zlabel('z');

plot3(point1(1,:), point1(2,:), point1(3,:))
hold on;
grid on;

R1 = rot.matrix_exp(w1, th(1));
T1 = [R1 [x1; y1; z1]; 0 0 0 1];
point1_changed = T1 * [point1; ones_added];
plot3(point1_changed(1,:), point1_changed(2,:), point1_changed(3,:))