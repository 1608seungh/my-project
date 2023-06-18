clear

road_shape

% Vehilce Parameters
a = 1.14; % distance c.g. to front axle (m)
L = 2.54; % wheel base (m)
m = 1500; % mass (kg)
Iz = 2420.0; % yaw moment of inertia (kg-m^2)
Cf = 44000*2; % cornering stiffness--front axle (N/rad)
Cr = 47000*2; % cornering stiffness-- rear axle (N/rad)
b = L-a;

%% control parameters
KP = 0.21;
KD = 0.16;
a_filt = 0.06;


TT= 0.04/a_filt;
% simulation parameters
time = 0:0.001:21;          % 20 seconds, 0.01sec step
OPTIONS = simset('Solver','ode45', 'MaxStep',1e-2,'RelTol',1e-3,'AbsTol',1e-3);

% simulation
sim('hw3c_sim', time, OPTIONS);
%%

x = car_pos.X.Data;  
y = car_pos.Y.Data;  
L_error = error.dy.Data;

figure(1)
clf
set(gcf, 'position', [100 200 1200 500]);
subplot(221)
plot(x1,y1,'k');
hold on
plot(x2,y2,'k');
plot(x0,y0,'r');
plot(x,y,'b--');
grid on
axis([0 450 0 300]);
plot([150 170 170 150 150], [40 40 60 60 40],'b','linewidth',2);
plot([200 220 220 200 200], [250 250 270 270 250],'b','linewidth',2);
text(180, 50, 'section 1');
text(230, 260, 'section 2');
xlabel('x (m)');
ylabel('y (m)');
title ('lane and trajectory');
mytext = ['Kp = ', num2str(KP), ', Kd = ', num2str(KD)];
text(350, 320, mytext);

subplot(245)
hold on
plot(x0,y0,'r');
plot(x,y,'b--');
plot(x1,y1,'k');
plot(x2,y2,'k');

grid on
axis([150 170 40 60]);
title('section 1');
hl=legend('lane center','vehicle');
set(hl,'location','southeast');

subplot(246)
plot(x1,y1,'k');
hold on
plot(x2,y2,'k');
plot(x0,y0,'r');
plot(x,y,'b--');
grid on
axis([200 220 250 270]);
title('section 2');

subplot(222)
plot(error.dy.Time, L_error);
hold on
plot(error.dy_f.Time, error.dy_f.Data);
grid on
xlabel('time (s)')
ylabel('Lateral error (m)');
grid on
title('tracking error')
legend('error','filtered error');
axis([0 25 -2 2]);

subplot(224)
plot(delta.Time, 17*delta.Data*180/pi,'linewidth',1); 
xlabel('Time (sec)')
ylabel('Steering Angle (degrees)')
grid on
title('Steering Anlge Input');
axis([0 25 -100 100]);
