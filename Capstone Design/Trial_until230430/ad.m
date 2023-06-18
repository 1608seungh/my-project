function AdT_bs = ad(T)

 p = T(1:3,4); % 3x1 Matrix p
 R = T(1:3,1:3); % 3x3 Matrix R
 p_mat = [0, -p(3), p(2);
        p(3), 0, -p(1);
        -p(2), p(1), 0]; % 3x3 Matrix [p]
 AdT_bs = [R, zeros(3); p_mat*R, R]; % 6x6 Matrix 

end


%p_mat = [0, -T_bs(3,4), T_bs(2,4);
%         T_bs(3,4), 0, -T_bs(1,4);
%        -T_bs(2,4), T_bs(1,4), 0] % 3x3 Matrix

% AdT_bs = [T(1:3,1:3), zero(3); p_mat*T(1:3,1:3), T(1:3,1:3)];