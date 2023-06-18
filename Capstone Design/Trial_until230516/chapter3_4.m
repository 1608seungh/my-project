classdef chapter3_4
    properties
        

        
    end

    methods 
        function so3mat = VecToso3(omg)
           
            so3mat = [0, -omg(3), omg(2); 
                      omg(3), 0, -omg(1); 
                      -omg(2), omg(1), 0];
        end

        function se3mat = VecTose3(V)
            se3mat = [VecToso3(V(1:3)), V(4:6); 0, 0, 0, 0];
        end

        function V = se3ToVec(se3mat)
            V = [se3mat(3, 2); se3mat(1, 3); se3mat(2, 1); se3mat(1: 3, 4)];
        end

        function omg = so3ToVec(so3mat)
            omg = [so3mat(3, 2); so3mat(1, 3); so3mat(2, 1)];
        end

        function T = FkinSpace(M, Slist, thetalist)
            T = M;
            for i = size(thetalist): -1 :1
                T = MatrixExp6(VecTose3(Slist(:,i) * thetalist(i))) * T;
            end
        end

        function T = MatrixExp6(se3mat)
            omgtheta = so3ToVec(se3mat(1: 3, 1: 3));
            if NearZero(norm(omgtheta))
                  T = [eye(3), se3mat(1: 3, 4); 0, 0, 0, 1];
            else
                  [omghat, theta] = AxisAng3(omgtheta);
                  omgmat = se3mat(1: 3, 1: 3) / theta; 
                  T = [MatrixExp3(se3mat(1: 3, 1: 3)), ...
                      (eye(3) * theta + (1 - cos(theta)) * omgmat ...
                       + (theta - sin(theta)) * omgmat * omgmat) ...
                         * se3mat(1: 3, 4) / theta;
                          0, 0, 0, 1];
            end
        end

        function AdT = Adjoint(T)
            [R, p] = TransToRp(T);
            AdT = [R, zeros(3); VecToso3(p) * R, R];
        end

        function S = ScrewToAxis(q, s, h)  
            S = [s; cross(q, s) + h * s];
        end

        function [R, p] = TransToRp(T)
            R = T(1: 3, 1: 3);
            p = T(1: 3, 4);
        end

        function expmat = MatrixLog6(T)
            [R, p] = TransToRp(T);
            omgmat = MatrixLog3(R);
            if isequal(omgmat, zeros(3))
                expmat = [zeros(3), T(1: 3, 4); 0, 0, 0, 0];
            else
                theta = acos((trace(R) - 1) / 2);
                expmat = [omgmat, (eye(3) - omgmat / 2 ...
                          + (1 / theta - cot(theta / 2) / 2) ...
                          * omgmat * omgmat / theta) * p;
                          0, 0, 0, 0];    
            end
        end

    end

end