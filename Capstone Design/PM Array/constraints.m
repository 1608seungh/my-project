function [C,Ceq] = constraints(x)
%% Initial condition
C = [];
Ceq = [];
x=reshape(x,[],25);
b_collect = [];
f_collect = [];

r_target=zeros(3,7);
r_target=[-1 0 2; 0 0 2; 1 0 2; 0 -1 2; 0 1 2; 0 0 1; 0 0 3]'*0.01; %타겟위치
%% Calculate b&f_collect
for r= r_target 
    b=zeros(3,1);
    f=zeros(3,1);
    for x_index = x
        rx=x_index(1:3);
        mx=x_index(4:6);
        b = b + ms(r-rx,mx);
    end
    b_collect = [b_collect b]; 

 for x_index = x
        rx=x_index(1:3);
        mx=x_index(4:6);
        f = f + ms_force(r-rx,b,mx);
 end

    f_collect = [f_collect f];
end

%% Inequality Constraints


%% Equality Constraints
Ceq(1) = norm(f_collect(:,2)); %Force at the point = 0
Ceq(2) = dot(f_collect(:,1),[1 0 0])-norm(f_collect(:,1)); %Force of left side >= 0
Ceq(3) = dot(f_collect(:,3),[1 0 0])+norm(f_collect(:,3)); %Force of right side <= 0
for i = 1:3
Ceq(i+3) = dot(b_collect(1:3,i),[1 0 0])-norm(b_collect(1:3,i));
end %Magnetic Field b's x >= 0
Ceq(7,1:25) = x(3,:); %Magnet Arrangement; z=0
for i = 1:5
    for k = 1:5
 Ceq(i+7,k) = x(1,k+5*(i-1)) + 0.03 - 0.015*(k-1);
 Ceq(i+7,k+5) = x(2,k+5*(i-1)) + 0.03 - 0.015*(i-1);
    end
end %Initial Magnet Arrangemnet
% for i = 1:25
% C(13+i) = norm(x(4:6,i)) - 1;
% end %Norm m2 = 1


end
