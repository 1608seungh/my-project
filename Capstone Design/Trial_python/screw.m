function screw = screw(w, q)

    w_mat = [0, -w(3), w(2); 
             w(3), 0, -w(1); 
            -w(2), w(1), 0]; 
    v = -cross(w, q);
    screw = [w_mat, v; 0 0 0 0];


end