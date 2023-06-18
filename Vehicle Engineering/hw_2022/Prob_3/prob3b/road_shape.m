s=0:0.1:600;
curv=zeros(size(s));

for k=1:length(curv)
    curv(k)=curvature(s(k));
end
th = cumtrapz(s,curv);
grad = tan(th);
x0 = cumtrapz(s, 1./sqrt(1+grad.^2));
y0 = cumtrapz(s, grad./sqrt(1+grad.^2));
x1 = 0*x0;
y1 = 0*y0;
x2 = 0*x0;
y2 = 0*y0;

theta = [0 atan2(diff(y0),diff(x0))]+pi/2;
for k=1:length(theta)
    R=[cos(theta(k)) -sin(theta(k)); sin(theta(k)) cos(theta(k))];
    temp= R*[1.5;0];
    x1(k)= x0(k)+temp(1,:);
    y1(k)= y0(k)+temp(2,:);
end

        
theta = [0 atan2(diff(y0),diff(x0))]-pi/2;
for k=1:length(theta)
    R=[cos(theta(k)) -sin(theta(k)); sin(theta(k)) cos(theta(k))];
    temp= R*[1.5;0];
    x2(k)= x0(k)+temp(1,:);
    y2(k)= y0(k)+temp(2,:);
end


z0 = x0*0;
z1 = x0*0;
z2 = x0*0;

%%
% figure(100)
% clf
% subplot(311)
% hold on
% % plot(x,y,'b','linewidth',1)
% plot(x1,y1,'k','linewidth',1)
% plot(x2,y2,'k','linewidth',1)
% 
% grid on
% ylabel('y (m)');
% xlabel('x (m)');
% 
% 
% subplot(312)
% plot(s, abs(1./(curv + 1e-10)));
% grid on
% xlabel('x (m)');
% ylabel('Radius (m)');
% ax=axis;
% axis([ax(1) ax(2) 0 500]);
% % set(gca,'yminortick','on','yminorgrid','on');
% % set(gca,'ytick',0:500:1500);
% 
% subplot(313)
% plot(s, curv);
% grid on
% xlabel('x (m)');
% ylabel('Curvature (1/m)');
% 
% 
% figure(101)
% clf
% hold on
% % plot(x,y,'b','linewidth',1)
% plot(x1,y1,'k','linewidth',1)
% plot(x2,y2,'k','linewidth',1)
% 
% grid on
% ylabel('y (m)');
% xlabel('x (m)');
% axis equal

%%

function res = curvature_basis(s)
    [p1,p2, dp1, dp2] = curv_factor();
    
    r1 = 0.2;%(1/dp1)^2;
    r2 = 0.3;

    s_vector = [0 p1-dp1*1.5, p1-0.5*dp1, p1+0.5*dp1, p1+1.5*dp1, p2-dp2*1.5, p2-0.5*dp2, p2+0.5*dp2, p2+1.5*dp2, 10000];
    c_vector = [0 0, r1, r1, 0, 0, -r2, -r2, 0, 0];
    res = interp1(s_vector, c_vector, s);
end

function res = curvature(s)
    res =0.7452*atan(9.678/2/78)*curvature_basis(s);
end

function [p1,p2, dp1, dp2] = curv_factor()
    p1 = 150;
    p2 = 400;        

    rfactor1 = 1;
    rfactor2 = 1;

    dp1 = 80*rfactor1;
    dp2 = 50*rfactor2;
end