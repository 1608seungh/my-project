clear all; clc; 

ACT = Func20230403;
BCT = robot_kinematic_0510test;

% Desired Pos & Orient 
p_d = [0.1; 0.1; 0.1];
T_sd = [eye(3) p_d; 0 0 0 1];

w1 = [0;0;1]; w2 = [1;0;0]; w3 = [1;0;0]; w4 = [1;0;0]; w5 = [0;1;0]; w6 = [0;0;1];
q1 = [0; 0; 0]; q2 = [0; 0; 0]; q3 = [0; 0.1036; 0];
q4 = [0; 0.2012; 0]; q5 = [0.0097; 0.2012; 0.0309]; q6 = [0.0229; 0.2833; 0.0309];

th = [0; pi/2; -pi/2; 0; 0; 0];

S1 = ACT.Screw(w1,q1); S2 = ACT.Screw(w2,q2); S3 = ACT.Screw(w3,q3); 
S4 = ACT.Screw(w4,q4); S5 = ACT.Screw(w5,q5); S6 = ACT.Screw(w6,q6);

M = [eye(3), [0.0097; 0.3583; 0.0309]; 0 0 0 1]; 

s1 = ACT.scr(w1,q1); s2 = ACT.scr(w2,q2); s3 = ACT.scr(w3,q3);
s4 = ACT.scr(w4,q4); s5 = ACT.scr(w5,q5); s6 = ACT.scr(w6,q6);

Slist = [s1 s2 s3 s4 s5 s6];
T_sb = BCT.FKinSpace(M, Slist, th)



% w = [[0;0;1], [1;0;0], [1;0;0], [1;0;0], [0;1;0], [0;0;1]];
% q = [[0;0;0], [0;0;0], [0;0.1036;0],
%      [0;0.2012;0], [0.0097;0.2012;0.0309], [0.0229;0.2833;0.0309]];

