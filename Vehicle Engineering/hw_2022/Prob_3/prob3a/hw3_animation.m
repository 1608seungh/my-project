time = car_pos.X.Time;  % simulation time
x = car_pos.X.Data;  
y = car_pos.Y.Data;  
heading = car_pos.phi.Data; % heading angle

time_swa = delta.Time;
swa = delta.Data;

figure(10)
clf

plot3(x1,y1,z1,'k','linewidth',4);
hold on
plot3(x2,y2,z2,'k','linewidth',4);
plot3(x0,y0,z0,'r','linewidth',1);
plot3(x,y,x*0,'b--','linewidth',1);

ax=gca;

ax.Projection = 'perspective';
ax.CameraViewAngle = 20;
ax.CameraTarget = [10 0 0];
ax.CameraPosition = [5 0 1];

axis equal
grid on   

% fill3([0 1 1 0 0],[15 15 15 15 15],[0 0 1 1 0],'r');

x_pre = x(1);
y_pre = y(1);
z_pre = 0;
yaw_pre = heading(1);

tttt = linspace(0, 2*pi, 37);
stwx = [0*tttt 0 0 0]+1;
stwy = [0.2*cos(tttt) -0.2 0 0];
stwz = [0.2*sin(tttt) 0 0 -0.2]+0.3;

h1 =plot3( 1+stwx, stwy, stwz,'k','linewidth',3);
%%
for kk=1:20:length(time)
    
    x_cur = x(kk);
    y_cur = y(kk);
    z_cur = 0;
    yaw_cur = heading(kk);

    dx = x_cur - x_pre;
    dy = y_cur - y_pre;
    dz = z_cur - z_pre;
    d_yaw = yaw_cur - yaw_pre;    

    del = interp1(time_swa, swa, time(kk))*17;
    RR1 = [cos(del) sin(del); -sin(del) cos(del)];
    temp1 = RR1*[stwy;stwz-0.3];
    
    RR2 = [cos(yaw_cur) -sin(yaw_cur); sin(yaw_cur) cos(yaw_cur)];
    temp2 = RR2*[stwx;temp1(1,:)];
    
    set(h1, 'xdata', x_cur+temp2(1,:),'ydata', y_cur+temp2(2,:),'zdata', temp1(2,:)+0.3);
    
    temp3 = RR2*[5 ;0];    
    ax.CameraTarget = [x_cur+temp3(1) y_cur+temp3(2) 0];
    ax.CameraPosition = [x_cur y_cur 0.6];
   
    ax=gca;

    drawnow
    grid on   

%     pause(0.1)
    axis equal
    x_pre = x_cur;
    y_pre = y_cur;
    z_pre = z_cur;
    yaw_pre = yaw_cur;
    grid on   
end

