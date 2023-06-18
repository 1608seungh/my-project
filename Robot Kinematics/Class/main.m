clear all; clc; clf;

tra = transformations;

w_hat = [1/sqrt(2); 1/sqrt(2); 0];
q = [1; 0; 0];
h = 0.5;
v = h*w_hat - cross(w_hat, q);
screw = [w_hat;v];
theta = pi/2;

T = eye(4);
plot_T(T, 'black')

for theta = 0:0.1:2*pi

    T01 = tra.screw_exp(screw, theta) * T;
    plot_T(T01, 'red')
    pause(0.1)

end

