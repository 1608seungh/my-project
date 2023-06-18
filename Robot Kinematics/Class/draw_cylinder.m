classdef draw_cylinder
    
    properties 

    end

    methods
       
        function draw_cyl(obj, z_d, point_desired)
        
            radius = 0.1;
            height = 0.3;
            
%             angle = 0:2*pi/100:2*pi;
%             x = radius*cos(angle);
%             y = radius*sin(angle);
%             z = ones(1,length(x))*height/2;
%             point_mat_lid = [x;y;z]; % 뚜껑 
%             point_mat_bottom = [x;y;-z]; % 바닥 
%             point_mat = [point_mat_lid, point_mat_bottom];
%             ones_added = ones(1,length(point_mat));
            
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

            rot = rotations;
            R = rot.matrix_exp(w_hat, theta);
            T = [R point_desired; 0 0 0 1];

            [X, Y, Z] = cylinder(radius);
            Z = (Z - 0.5) * height;

            XYZ1 = R * [X(1,:); Y(1,:); Z(1,:)];
            XYZ2 = R * [X(2,:); Y(2,:); Z(2,:)];

            X = [XYZ1(1,:); XYZ2(1,:)] + point_desired(1);
            Y = [XYZ1(2,:); XYZ2(2,:)] + point_desired(2);
            Z = [XYZ1(3,:); XYZ2(3,:)] + point_desired(3);

            surf(X,Y,Z, 'FaceColor', 'blue')
            hold on
            fill3(X(1,:),Y(1,:),Z(1,:), 'blue')
            fill3(X(2,:),Y(2,:),Z(2,:), 'blue')

            
%             % point_mat_changed = R * point_mat;
%             point_mat_changed = T * [point_mat; ones_added];
%             
%              plot3(point_mat(1,:), point_mat(2,:), point_mat(3,:)) % 기본형 
%             plot3(point_mat_changed(1,:), point_mat_changed(2,:), point_mat_changed(3,:))
%             hold on 
%              axis equal;
        end

    end
end