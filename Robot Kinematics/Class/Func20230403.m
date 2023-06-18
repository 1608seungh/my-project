classdef Func20230403

    properties 

    end

    methods 
        function w_mat = Skew(obj, w)
            w_mat = [0,   -w(3), w(2);
                     w(3),  0,  -w(1); 
                    -w(2), w(1),   0];
        end

        function S = Screw(obj, w, q)
            v = -obj.Skew(w)*q;
            S = [obj.Skew(w), v; 0 0 0 0]; % 4x4 Matrix
        end

        function AdT_bs = Ad(obj, T) % Make an Adjoint Matrix
            p = T(1:3,4); % 3x1 Matrix p
            R = T(1:3,1:3); % 3x3 Matrix R
            AdT_bs = [R, zeros(3); obj.Skew(p)*R, R]; % 6x6 Matrix
        end

        function output = BodyTwist(obj, V_bmat) 
            output = [-V_bmat(2,3); V_bmat(1,3); -V_bmat(1,2); V_bmat(1:3,4)]; % 6x1 Matrix 
        end

        function twist = S2twist(obj, se3)
            w_mat = se3(1:3,1:3);
            v = se3(1:3,4);

            w1 = -w_mat(2,3);
            w2 = w_mat(1,3);
            w3 = -w_mat(1,2);
            twist = [w1;w2;w3;v]; % Space Twist // 6x1 Matrix
            end

    end

end