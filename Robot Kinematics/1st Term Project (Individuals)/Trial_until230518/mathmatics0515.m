classdef mathmatics0515
    properties 
        
    end


    methods
        function output = Skew(obj, w)
            output = [   0  -w(3)   w(2);
                       w(3)    0   -w(1);
                      -w(2)  w(1)     0];
        end

        function out = BodyTwist(obj, V_bmat)
            w_b = [V_bmat(3, 2); V_bmat(1, 3); V_bmat(2, 1)];
            v_b = V_bmat(1:3, 4);           
            out = [w_b; v_b];
        end

        function R = matrix_exp(obj, w_hat, theta)
            R = eye(3) + sin(theta) * obj.Skew(w_hat) + (1-cos(theta)) * obj.Skew(w_hat) * obj.Skew(w_hat);
        end

        function T = screw_exp(obj, screw, theta)
            w_hat = screw(1:3);
            v = screw(4:6);

            w_hat_skew = obj.Skew(w_hat);

            R = obj.matrix_exp(w_hat, theta);
            G_theta = eye(3)*theta + (1-cos(theta)) * w_hat_skew + (theta-sin(theta)) * w_hat_skew^2;

            T = [R G_theta*v
                0 0 0 1];

        end

        function AdT = Ad(obj, T)
            
            R = T(1:3, 1:3);
            p = T(1:3, 4);
            p_skew = obj.Skew(p);

            AdT = [R zeros(3,3); p_skew*R R];
            
        end
    end

end