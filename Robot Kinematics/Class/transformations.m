classdef transformations
    properties
        
        a = rotations;
        
    end

    methods 
        function T = screw_exp(obj, screw, theta)
            w_hat = screw(1:3);
            v = screw(4:6);

            w_hat_skew = obj.a.Skew(w_hat);

            R = obj.a.matrix_exp(w_hat, theta);
            G_theta = eye(3)*theta + (1-cos(theta)) * w_hat_skew + (theta-sin(theta)) * w_hat_skew^2;

            T = [R G_theta*v
                0 0 0 1];

        end

        function AdT = Ad(obj, T)
            
            R = T(1:3, 1:3);
            p = T(1:3, 4);
            p_skew = obj.a.Skew(p);

            AdT = [R zeros(3,3); p_skew*R R];
            
        end

    end

end
