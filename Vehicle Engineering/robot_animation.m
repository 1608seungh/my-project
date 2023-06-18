clear all; clc;
hold off;

x = 10; y =10; %initial positiob 
r = 27.5; % robot radius
head  = 0; %initial head angle
L     = 50; %wheel base
h     = 50; %threshold
theta1= 0;
theta2= 0;
jj    = 0;
vl    = 1; %왼쪽바퀴 속도
vr    = 2;%오른쪽바퀴 속도
ang   = 0:0.1:2*pi+0.2; 

for ii = 0:0.1:50  
  if(vl==vr)
      x = x+vl*cos(theta1+theta2);
      y = y+vl*sin(theta1+theta2);
  else    
      R=L/2*(vl+vr)/(vr-vl);
      w = (vl+vr)/2/R; %psi
      theta2 = w*ii;
      x = x+2*R*cos((2*theta1+theta2)/2)*sin(theta2/2); %x position change
      y = y+2*R*sin((2*theta1+theta2)/2)*sin(theta2/2); %x position change
  end
   
  rx = x + r * cos(ang); ry = y + r * sin(ang); % robot drawing 
  head =theta1+ theta2; %change head angle
  plot(rx,ry);  % draw robot body
  hold on;

  line([-100 500], [-100 -100]), line([500 500],[-100 500]), line([-100 500],[500 500]), line([-100 -100],[-100 500]); % boundary
  line([x x+r*cos(head)], [y y+r*sin(head)]); % mark head 
  line([x+(r-2.5)*cos(head+45) x+(r+50)*cos(head+45)], [y+(r-2.5)*sin(head+45) y+(r+50)*sin(head+45)]); % left IR sensor
  line([x+(r-2.5)*cos(head-45) x+(r+50)*cos(head-45)], [y+(r-2.5)*sin(head-45) y+(r+50)*sin(head-45)]); % right IR sensor
  axis([-150 550 -150 550]) 
  
  lirx = x+(r+h)*cos(head+45); % sensor thresould
  liry = y+(r+h)*sin(head+45);
  rirx = x+(r+h)*cos(head-45);
  riry = y+(r+h)*sin(head-45);
  
  m=(500-lirx)*(-100-lirx)*(500-liry)*(-100-liry); % sensor sign
  n=(500-rirx)*(-100-rirx)*(500-riry)*(-100-riry);
  
  if(m<0) % left sensor on = stop % turn right
      vl = 0;
      vr = 0;
      jj = jj+1;
      theta1 = theta1-jj/10;
  end
  if(n<0) % right sensor on = stop % turn left
      vr = 0;
      vl = 0;
      jj = jj+1;
      theta1 = theta1+jj/10;
  end
  if(m>0 && n>0) % sensor off = run
      vr = 10;
      vl = 10;
      jj = 0;
  end
   
  pause(0.05); %you can control the speed of animation with this 
  hold off;
end