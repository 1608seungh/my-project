clear all; clc;

PM = Func_0525temp;

w1 = [0;0;1];
v1 = [0;1;0];
s1 = [w1;v1];
th = 0.5;

R = PM.matrix_exp(w1, th)
T1 = PM.screw_exp(s1, th)

M = [eye(3),[0; 0.166; -0.01]; 0, 0, 0, 1];

M_ = T1 * M
A = PM.Ad(M_)

B = PM.BodyTwist(M_)