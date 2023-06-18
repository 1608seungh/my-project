% fwd_kin_test
clear all; clc;

for t = 0:0.1:5
   theta = [0.2*t; -0.4*t; 0.3*t];
    z_hat = [0;0;1];
    
    w1 = z_hat;
    q1 = [0;0;1];
    v1 = -cross(w1,q1);
    s1 = [w1;v1];
    M1 = [eye(3) q1; 0 0 0 1];
    
    w2 = z_hat;
    q2 = [0;1;1];
    v2 = -cross(w2,q2);
    s2 = [w2;v2];
    M2 = [eye(3) q2; 0 0 0 1];
    
    w3 = z_hat;
    q3 = [0;2;1];
    v3 = -cross(w3,q3);
    s3 = [w3;v3];
    M3 = [eye(3) q3; 0 0 0 1];
    
    M4 = [eye(3) [0;3;1]; 0 0 0 1];
    
    tra = transformations;
    exp1 = tra.screw_exp(s1, theta(1));
    exp2 = tra.screw_exp(s2, theta(2));
    exp3 = tra.screw_exp(s3, theta(3));
    
    M2_ = exp1*M2;
    M3_ = exp1*exp2*M3;
    M4_ = exp1*exp2*exp3*M4;
    
    w1_ = M1(1:3, 3);
    q1_ = M1(1:3, 4);
    
    w2_ = M2_(1:3, 3);
    q2_ = M2_(1:3, 4);
    
    w3_ = M3_(1:3, 3);
    q3_ = M3_(1:3, 4);
    
    q4_ = M4_(1:3, 4);
    
    dra = draw_cylinder;
    dra.draw_cyl(w1_, q1_);
    hold on
    dra.draw_cyl(w2_, q2_);
    dra.draw_cyl(w3_, q3_);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([-4, 4])
    ylim([-4, 4])
    zlim([0, 2])
    
    line = [q1_, q2_, q3_, q4_];
    plot3(line(1,:), line(2,:), line(3,:), '-o')
    pause(0.01)
    clf

end