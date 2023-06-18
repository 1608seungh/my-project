s=0:0.1:600;
curv=zeros(size(s));

for k=1:length(curv)
    curv(k)=curvature(s(k));
end
th = cumtrapz(s,curv);
grad = tan(th);
x = cumtrapz(s, 1./sqrt(1+grad.^2));
y = cumtrapz(s, grad./sqrt(1+grad.^2));
x1 = 0*x;
y1 = 0*y;
x2 = 0*x;
y2 = 0*y;

phi_ref = [0 atan2(diff(y),diff(x))];

theta = [0 atan2(diff(y),diff(x))]+pi/2;
for k=1:length(theta)
    R=[cos(theta(k)) -sin(theta(k)); sin(theta(k)) cos(theta(k))];
    temp= R*[1.5;0];
    x1(k)= x(k)+temp(1,:);
    y1(k)= y(k)+temp(2,:);
end

        
theta = [0 atan2(diff(y),diff(x))]-pi/2;
for k=1:length(theta)
    R=[cos(theta(k)) -sin(theta(k)); sin(theta(k)) cos(theta(k))];
    temp= R*[1.5;0];
    x2(k)= x(k)+temp(1,:);
    y2(k)= y(k)+temp(2,:);
end

        
z1 = x*0;
z2 = x*0;


x0=x;
y0=y;
z0=x*0;

%%
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