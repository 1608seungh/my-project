function Skew_mat = Skew(w)

Skew_mat = [0, -w(3), w(2); 
            w(3), 0, -w(1); 
           -w(2), w(1), 0];
end