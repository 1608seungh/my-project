function Js = SpaceJacobi(w,q)

w_mat = [0,   -w(3), w(2);
         w(3),  0,  -w(1); 
        -w(2), w(1),   0];
v = -w_mat*q; 

Js = [w1,w2,w3,w4,w5,w6; v1,v2,v3,v4,v5,v6]


end