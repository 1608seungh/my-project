classdef Term_Project_Classdef
    properties
        
        mat = mathmatics0515;

        S; v; w; q; M; M_end; Trs; th; 
        Js; Jb; T_bs; T_sb; T_bd; T_sd; 
        V_bmat; V_b; w_b; v_b; w_; q_;
 
    end

    methods
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
                exp(:, :, i) = obj.mat.screw_exp(obj.S(:, i), obj.th(i));
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
                obj.Js(:,i) = obj.mat.Ad(T) * obj.S(:,i);
            end

            % Body Jacobian
            obj.T_sb = obj.Trs(:, :, end);
            obj.T_bs = inv(obj.T_sb);
            obj.Jb = obj.mat.Ad(obj.T_bs)*obj.Js;

            % Body Twist
            obj.T_bd = obj.T_bs * obj.T_sd;
            obj.V_bmat = logm(obj.T_bd)*0.25;
            obj.V_b = obj.mat.BodyTwist(obj.V_bmat);
            obj.w_b = obj.V_b(1:3); obj.v_b = obj.V_b(4:6);

            % Inverse Kinematic
            dth = pinv(obj.Jb)*obj.V_b;
            obj.th = obj.th + dth;
%             th_deg = rad2deg(obj.th)'; 

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
            xlim([-10 70])
            ylim([-10 70])
            xticks([-10:10:70])
            yticks([-10:10:70])
        end

        function obj = draw_cyl(obj, z_d, point_desired)
        
            radius = 1.5;
            height = 9;
            
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

            R = obj.mat.matrix_exp(w_hat, theta);
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
                grid on
                xlabel('x'); ylabel('y'); zlabel('z');
                xlim([-10 80])
                ylim([-10 80])
                zlim([-10 80])
          
               plot3(obj.q_(1,:), obj.q_(2,:), obj.q_(3,:), 'color', 'blue')
           
        end         

%         function obj = draw_xyz(obj)
%             p0 = obj.Trs(1:3, 4, 1); 
%             p1 = obj.Trs(1:3, 4, 2); 
%             p2 = obj.Trs(1:3, 4, 3); 
%             p3 = obj.Trs(1:3, 4, 4); 
%             p4 = obj.Trs(1:3, 4, 5); 
%             p5 = obj.Trs(1:3, 4, 6); 
%             p6 = obj.Trs(1:3, 4, end);
% 
%             title('3D Visualization')
%             axis equal
%         
%             line([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)])
%             line([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)])
%             line([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)])
%             line([p3(1), p4(1)], [p3(2), p4(2)], [p3(3), p4(3)])
%             line([p4(1), p5(1)], [p4(2), p5(2)], [p4(3), p5(3)])
%             line([p5(1), p6(1)], [p5(2), p6(2)], [p5(3), p6(3)])
%             
%             grid on
%             xlabel('x'); ylabel('y'); zlabel('z');
%             xlim([-10 80])
%             ylim([-10 80])
%             zlim([-10 80])
%         end

    end

end