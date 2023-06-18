classdef draw_cylinder
    
    properties 

    end

    methods
       
        function draw_cyl(obj, z_d, point_desired)
        
            radius = 0.002;
            height = 0.006;
            
            angle = 0:2*pi/100:2*pi;
            x = radius*cos(angle);
            y = radius*sin(angle);
            z = ones(1,length(x)) * height/2;
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
                theta = asin(a_norm);
                w_hat = a/a_norm;

            end

            rot = rotations;
            R = rot.matrix_exp(w_hat, theta);
            T = [R point_desired; 0 0 0 1];
            
            % point_mat_changed = R * point_mat;
            point_mat_changed = T * [point_mat; ones_added];
            
%             plot3(point_mat(1,:), point_mat(2,:), point_mat(3,:)) % 기본형 
            plot3(point_mat_changed(1,:), point_mat_changed(2,:), point_mat_changed(3,:))
%            hold on 
%             axis equal;
        end

    end
end