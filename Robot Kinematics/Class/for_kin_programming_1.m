% clear all; clc; 
% clf;

tra = transformations;

M = [eye(3) [3;0;0]; 0 0 0 1];

screw_3 = [0;0;1;0;-2;0];
screw_2 = [0;0;0;1;0;0];
screw_1 = [0;0;1;0;0;0];

theta_1 = 0;
theta_2 = -0.5
theta_3 = pi/2;

for theta_1 = 0:-0.1:-pi/4

    E3 = tra.screw_exp(screw_3, theta_3);
    E2 = tra.screw_exp(screw_2, theta_2);
    E1 = tra.screw_exp(screw_1, theta_1);
    
    T = E1 * E2 * E3 * M;

    plot_T(T, 'black')
    pause(0.1);

end

% 
% for theta = 0:0.1:2*pi
% 
%     T01 = tra.screw_exp(screw, theta) * T;
%     plot_T(T01, 'red')
%     pause(0.1)
% 
% end
