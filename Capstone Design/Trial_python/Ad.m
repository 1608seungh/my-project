function Adjoint = Ad(T)

p = T(1:3, 4);
R = T(1:3, 1:3);
p_mat = [0, -p(3), p(2); 
         p(3), 0, -p(1); 
        -p(2), p(1), 0];
Adjoint = [R, zeros(3); p_mat*R, R];

end