function twist = s2twist(se3)
w_mat = se3(1:3,1:3);
v = se3(1:3,4);

w1 = -w_mat(2,3);
w2 = w_mat(1,3);
w3 = -w_mat(1,2);
twist = [w1;w2;w3;v]; % Space Twist // 6x1 Matrix

end