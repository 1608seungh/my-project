clear a;
clear all; clc;

a = arduino();

s1 = servo(a,"D2", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s2 = servo(a,"D3", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s3 = servo(a,"D9", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s4 = servo(a,"D8", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s5 = servo(a,"D4", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
s6 = servo(a,"D5", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);

writePosition(s1, 90/180);
writePosition(s2, 90/180);
writePosition(s3, 90/180);
writePosition(s4, 90/180);
writePosition(s5, 90/180);
writePosition(s6, 90/180);

th = [readPosition(s1); readPosition(s2); readPosition(s3); readPosition(s4); readPosition(s5)];
th = th * 180;


% for angle = 0:0.1:1.2
%     writePosition(s3,angle)
%     current_pos = readPosition(s3);
%     current_pos = current_pos * 180
%     pause(2)
% end