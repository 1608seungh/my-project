classdef rotations

    properties 

    end

    methods 
        function output = RotX(obj,theta)
        output = [1 0 0
                0 cos(theta) -sin(theta)
                0 sin(theta) cos(theta)];
        end

        function output = RotY(obj,theta)
        output = [cos(theta) 0 sin(theta)
                   0 1 0
               -sin(theta) 0 cos(theta)];
        end

        function output = RotZ(obj,theta)
        output = [cos(theta) -sin(theta) 0
                sin(theta) cos(theta) 0
                0 0 1];
        end

        function output = Skew(obj, w)
            output = [   0  -w(3)   w(2);
                       w(3)    0   -w(1);
                      -w(2)  w(1)     0];
        end

        function R = matrix_exp(obj, w_hat, theta)
            
            R = eye(3) + sin(theta) * obj.Skew(w_hat) + (1-cos(theta)) * obj.Skew(w_hat) * obj.Skew(w_hat);

        end

    end

end