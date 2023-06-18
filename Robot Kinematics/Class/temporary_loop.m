% Drawing Cylinder & Rotating

radius = 0.3;
height = 0.9;

angle = 0:2*pi/100:2*pi;
x = cos(angle);
y = sin(angle);
z = ones(1,length(x)) * height/2;
point_mat_lid = [x;y;z]; % 뚜껑 
point_mat_bottom = [x;y;-z]; % 바닥 
point_mat = [point_mat_lid, point_mat_bottom];
ones_added = ones(1,length(point_mat));

z_hat = [0;0;1];
z_d = [1;0;0];

point_desired = [1;1;1];

a = cross(z_hat, z_d);
a_norm = norm(a);
theta = asin(a_norm);
w_hat = a/a_norm;
rot = rotations;
R = rot.matrix_exp(w_hat, theta);
T = [R point_desired; 0 0 0 1];


% point_mat_changed = R * point_mat;
point_mat_changed = T * [point_mat; ones_added];

plot3(point_mat_changed(1,:), point_mat_changed(2,:), point_mat_changed(3,:))
% plot3(point_mat(1,:), point_mat(2,:), point_mat(3,:))
% hold on;
% plot3(point_mat_changed(1,:), point_mat_changed(2,:), point_mat_changed(3,:))

axis equal

% clear all; clc;
% % Drawing
% 
% x = 0:pi/100:10*pi;
% y = sin(x);
% z = cos(x);
% plot3(x,y,z,'--','Linewidth', 2.5)
% xlabel('x')
% ylabel('y')
% zlabel('z')

% for loop
% clear all; clc;
% % a = [1, 3, 100, 7, 9];
% a = transpose(1:0.1:100);
% b = a * 2;
% 
% for a_index = a
% %     disp(a_index)
% %     disp(a_index * 2)
% end
% 
% i = 0;
% 
% while(i < 40)
%     i = i + 1;
%     disp(i)
%     pause(0.1)
% end

% v = [1,3,4,5,6,7,8,6,4,2,32,2,6,54,5,3,2,3,1];
% v(3:end)
% v(1:5)
% v(1:end-7)
% 
% M = [1,3,4,5,3;
%     3,34,35,63,1;
%     3,4,5,6,7];
% M(2,3)
% M(1:2,1:2:5)
% M([1,3], [2,4])

