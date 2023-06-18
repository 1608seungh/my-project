classdef robot_kinematic_0510test
    properties 
    
    end

    methods
        function so3mat = VecToso3(obj, w)
            so3mat = [0, -w(3), w(2); 
                      w(3), 0, -w(1); 
                      -w(2), w(1), 0];
        end

        function omg = so3ToVec(obj, so3mat)
            omg = [obj.so3mat(3, 2); obj.so3mat(1, 3); obj.so3mat(2, 1)];
        end
        
        function se3mat = VecTose3(obj, V)
            se3mat = [obj.VecToso3(V(1:3)), V(4:6); 0, 0, 0, 0];
        end
        
        function judge = NearZero(near)
            judge = norm(near) < 1e-6;
        end

        function [omghat, theta] = AxisAng3(expc3)
            theta = norm(expc3);
            omghat = expc3 / theta;
        end

        function  R = MatrixExp3(so3mat)
            omgtheta = obj.so3ToVec(so3mat);
            if obj.NearZero(norm(omgtheta))
                R = eye(3);
            else
              [omghat, theta] = obj.AxisAng3(omgtheta);
              omgmat = obj.so3mat / theta;
              R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
            end
        end


        function Tr = MatrixExp6(obj, se3mat)
            omgtheta = obj.so3ToVec(obj.se3mat(1: 3, 1: 3));
            if obj.NearZero(norm(omgtheta))
                Tr = [eye(3), obj.se3mat(1: 3, 4); 0, 0, 0, 1];
            else
                [omghat, theta] = obj.AxisAng3(omgtheta);
                 omgmat = obj.se3mat(1: 3, 1: 3) / theta; 
                  Tr = [obj.MatrixExp3(obj.se3mat(1: 3, 1: 3)), ...
                       (eye(3) * theta + (1 - cos(theta)) * omgmat ...
                       + (theta - sin(theta)) * omgmat * omgmat) ...
                        * obj.se3mat(1: 3, 4) / theta;
                          0, 0, 0, 1];
            end
        end


        function T = FKinSpace(obj, M, Slist, th)         
            T = M;
            for i = size(th): -1: 1
               T = obj.MatrixExp6(obj.VecTose3(Slist(:, i) * th(i))) * T;
            end
        end

    end

end 