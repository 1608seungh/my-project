function V_b = BodyTwist(V_bmat)

V_b = [-V_bmat(2,3); V_bmat(1,3); -V_bmat(1,2); V_bmat(1:3,4)]; % 6x1 Matrix 

end

