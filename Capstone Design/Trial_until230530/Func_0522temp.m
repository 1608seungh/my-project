classdef Func_0522temp
    properties      
        S; v; w; q; M; M_end; Trs; th; 
        Js; Jb; T_bs; T_sb; T_bd; T_sd; 
        V_bmat; V_b; w_; q_;
    end

    methods
        function output = Skew(obj, w)
            output = [   0  -w(3)   w(2);
                       w(3)    0   -w(1);
                      -w(2)  w(1)     0];   
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

        function out = BodyTwist(obj, V_bmat)
            w_b = [V_bmat(3, 2); V_bmat(1, 3); V_bmat(2, 1)];
            v_b = V_bmat(1:3, 4);           
            out = [w_b; v_b];
        end

        function obj = calculate(obj)

            % Screw Representation
            obj.v = zeros(3, length(obj.w));
            obj.S = zeros(6, length(obj.w));
            obj.M = zeros(4, 4, length(obj.w)+1);
            obj.M(:,:,end) = obj.M_end;
            exp = zeros(4, 4, length(obj.w));

            i = 0;
            for q_idx = obj.q
               i = i + 1;
                obj.v(:, i) = -cross(obj.w(:, i), q_idx);
                obj.S(:, i) = [obj.w(:, i); obj.v(:, i)];
                obj.M(:, :, i) = [eye(3) q_idx; 0 0 0 1];
                exp(:, :, i) = obj.screw_exp(obj.S(:, i), obj.th(i));
            end
            
            % Forward Kinematics
            obj.Trs(:, :, 1) = obj.M(:, :, 1);
            for i = 1:length(obj.w)
                obj.Trs(:, :, i+1) = obj.M(:, :, i+1);
                for j = i:-1:1
                    obj.Trs(:, :, i+1) = exp(:, :, j) * obj.Trs(:, :, i+1);
                end
            end
        
            % Space Jacobian
            obj.Js = zeros(6, length(obj.w));
            obj.Js(:, 1) = obj.S(:, 1);
            for i = 2:length(obj.w)
                T = eye(4);
                for j = 1:i-1
                    T = T*exp(:, :, j);
                end
                obj.Js(:,i) = obj.Ad(T) * obj.S(:,i);
            end

            % Body Jacobian
            obj.T_sb = obj.Trs(:, :, end);
            obj.T_bs = inv(obj.T_sb);
            obj.Jb = obj.Ad(obj.T_bs)*obj.Js;

            % Body Twist
            obj.T_bd = obj.T_bs * obj.T_sd;
            obj.V_bmat = logm(obj.T_bd) * 0.07;
            obj.V_b = obj.BodyTwist(obj.V_bmat);

            % After Transformation
            obj.w_ = zeros(size(obj.w));
            obj.q_ = zeros(size(obj.q));
        
            for i = 1:length(obj.w) + 1
                 obj.w_(:,i) = obj.Trs(1:3, 3, i);
                 obj.q_(:,i) = obj.Trs(1:3, 4, i);
            end

        end

        function obj = draw_xy(obj)
            title('XY Plane Visualization')
            hold on
            grid on
            xlabel('x'); ylabel('y');
            axis equal
%             xlim([-10 10])
%             ylim([-10 10])
%             xticks([-10:2:10])
%             yticks([-10:2:10])

            xlim([-0.3 0.3])
            ylim([-0.3 0.3])
            xticks([-0.3:0.1:0.3])
            yticks([-0.3:0.1:0.3])
        end

        function obj = draw_cyl(obj, z_d, point_desired)
%         
%             radius = 0.3;
%             height = 1;

            radius = 0.006;
            height = 0.02;
            
            angle = 0:2*pi/100:2*pi;
            x = radius*cos(angle);
            y = radius*sin(angle);
            z = ones(1,length(x))*height/2;
            point_mat_lid = [x;y;z]; % 뚜껑 
            point_mat_bottom = [x;y;-z]; % 바닥 
            point_mat = [point_mat_lid, point_mat_bottom];
            ones_added = ones(1,length(point_mat));
            
            z_hat = [0;0;1];
            
            a = cross(z_hat, z_d);
            a_norm = norm(a);

            if a_norm == 0
                w_hat = z_hat;
                theta = 0;

            else
                theta = real(asin(a_norm));
                w_hat = a/a_norm;

            end

            R = obj.matrix_exp(w_hat, theta);
            T = [R point_desired; 0 0 0 1];
            
            % point_mat_changed = R * point_mat;
            point_mat_changed = T * [point_mat; ones_added];
            
%             plot3(point_mat(1,:), point_mat(2,:), point_mat(3,:)) % 기본형 
            plot3(point_mat_changed(1,:), point_mat_changed(2,:), point_mat_changed(3,:))
%            hold on 
%             axis equal;
        end

        function obj = draw_xyz(obj)
                for i = 1:length(obj.w)          
                    obj.draw_cyl(obj.Js(1:3, i), obj.q_(:,i));
                    hold on;
                end
            
                title('3D Visualization')
                axis equal
                grid on;
                xlabel('x'); ylabel('y'); zlabel('z');
                xlim([-0.3 0.3])
                ylim([-0.3 0.3])
                zlim([-0.1 0.3])

%                 xlim([-10 10])
%                 ylim([-10 10])
%                 zlim([-5 15])
               plot3(obj.q_(1,:), obj.q_(2,:), obj.q_(3,:), 'color', 'blue')

                Tsb = obj.Trs(:,:,end);
                obj.draw_frame(Tsb)
           
        end        


        function draw_frame(obj, T)

            R = T(1:3, 1:3);
            p = T(1:3, 4);
            frame_size = 0.005;

            for i = 1:3
                 p_ = p + R(:,i)*frame_size; % x,y,z axis 방향으로 그림 
                point_set = [p, p_];

                plot3(point_set(1,:), point_set(2,:), point_set(3,:), 'color', 'black') 
                hold on;
            end
        end

    end
end