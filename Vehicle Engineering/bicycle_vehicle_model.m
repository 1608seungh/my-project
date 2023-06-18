clear all; clc;
hold off;

x = 0; 
y = 10; 
psi = 0; 

lf = 1; 
lr = 1;
l = lr+lf;

vel = 10*1000/3600; 
dt = 0.1;
beta = 0;

for i= 1:157
    del= 21+0.44*i;
    
    x= x+vel*cos(psi*pi/180+beta)*dt;
    y= y+vel*sin(psi*pi/180+beta)*dt;
    psi= psi+vel/(lf+lr)*tan(del*pi/180)*cos(beta)*dt;
    beta= atan(lr/(lf+lr)*tan(del*pi/180));

    plot(x,y,'-o');
    xlabel('x');
    ylabel('y');
    grid

    hold on;
   
    line([0 0], [0 40]),line([0 40], [40 40]), line([40 40], [40 0]),line([40 0], [0 0]);
    line([0 20], [20 20]),line([20 20], [20 40]); 
    axis([-2.5 42.5 -2.5 42.5])
end
