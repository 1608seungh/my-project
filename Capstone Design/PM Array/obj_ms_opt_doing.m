clear all
clc

tic
%% initial point setting
x0 = zeros(6,25);

for i=1:5 %x축 조기조건
    for k = 1:5
       x0(1,k+5*(i-1)) = -0.03 + 0.015*(k-1);
       x0(2,k+5*(i-1)) = -0.03 + 0.015*(i-1);
    end
       
    x0(4:6,1+5*(i-1)) = [1 0 0]';
    x0(4:6,2+5*(i-1)) = [0 0 1]';
    x0(4:6,3+5*(i-1)) = [-1 0 0]';
    x0(4:6,4+5*(i-1)) = [0 0 -1]';
    x0(4:6,5+5*(i-1)) = [1 0 0]';
    
end

%% targeted force at specific location

r_target=zeros(3,7);
r_target=[-1 0 2; 0 0 2; 1 0 2; 0 -1 2; 0 1 2; 0 0 1; 0 0 3]'*0.01; %타겟위치

fcn_obj = @(x)obj_ms_opt(r_target,x);

%% optimization option
tol = 1.0e-6;
iter = 1.0e+13;
options = optimoptions(@fmincon, 'Display', 'iter', 'PlotFcn', @optimplotfval, 'TolX', tol, 'TolFun', tol, 'TolCon', tol, 'Algorithm', 'sqp', 'MaxFunctionEvaluations', iter);

[x,fval,exitflag,output]= fmincon(fcn_obj,x0,[],[],[],[],[],[],@(x) constraints(x),options)

%% display

x=reshape(x,1,[]);
x=reshape(x,[],25);

for i = 1:25
x(4:6,i) = x(4:6,i)/norm(x(4:6,i)) ; %to unit vector
end

[~, b_collect, f_collect] = obj_ms_opt(r_target,x)

Fx123 = [f_collect(:,1:3)]
Fy425 = [f_collect(:,4) f_collect(:,2) f_collect(:,5)]
Fz627 = [f_collect(:,6) f_collect(:,2) f_collect(:,7)]

toc