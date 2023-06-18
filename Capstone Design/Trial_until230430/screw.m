function S = screw(w,q)

w_mat = [0,   -w(3), w(2);
         w(3),  0,  -w(1); 
        -w(2), w(1),   0];
v = -w_mat*q; 
S = [w_mat, v; 0 0 0 0];


end