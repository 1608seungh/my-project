classdef serial_fwd_kin
    properties
        tra = transformations;
        dra = draw_cylinder;
        s; v; w; q; M;
        M_; w_; q_; theta; M_end;
        Js; Jb;

    end

    methods
        function obj = calculate(obj)
            obj.v = zeros(3, length(obj.w)); 
            obj.s = zeros(6, length(obj.w));
            obj.M = zeros(4,4, length(obj.w)+1);
            obj.M(:,:,end) = obj.M_end;
        
            exp_ = zeros(4, 4, length(obj.w));
        
            i = 0;
            for q_idx = obj.q   
        
                i = i+1;
        
                obj.v(:,i) = -cross(obj.w(:,i), q_idx);
                obj.s(:,i) = [obj.w(:,i); obj.v(:,i)];
                obj.M(:,:,i) = [eye(3), q_idx; 0 0 0 1]; % 이거를 좀 손 봐야 함 
                exp_(:,:,i) = obj.tra.screw_exp(obj.s(:,i), obj.theta(i));
        
            end

            
            obj.M_(:,:,1) = obj.M(:,:,1);
        
            for i = 1:length(obj.w)
                obj.M_(:,:,i+1) = obj.M(:,:,i+1);
                for j = i:-1:1
                    obj.M_(:,:,i+1) = exp_(:,:,j)*obj.M_(:,:,i+1);
                end
            end
            
            % Space Jacobian 
            obj.Js = zeros(6, length(obj.w));
            obj.Js(:,1) = obj.s(:,1);

            for i = 2: length(obj.w)

                T = eye(4);

                for j = 1 : i-1
                    T = T * exp_(:, :, j);
                end
                
                obj.Js(:,i) = obj.tra.Ad(T)*obj.s(:,i);

            end

            Tsb = obj.M_(:,:,end);
            Tbs = inv(Tsb);
            obj.Jb = obj.tra.Ad(Tbs)*obj.Js;

            obj.w_ = zeros(size(obj.w));
            obj.q_ = zeros(size(obj.q));
        
            for i = 1:length(obj.w) + 1
                 obj.w_(:,i) = obj.M_(1:3, 3, i);
                 obj.q_(:,i) = obj.M_(1:3, 4, i);
        
            end
        end

        function obj = draw_linkage(obj)
                for i = 1:length(obj.w)          
                    obj.dra.draw_cyl(obj.Js(1:3, i), obj.q_(:,i));
                    hold on;
                end
            
                xlabel('x'); ylabel('y'); zlabel('z');
                xlim([-5, 5])
                ylim([-5, 5])
                zlim([-1, 4])
          
                plot3(obj.q_(1,:), obj.q_(2,:), obj.q_(3,:), '-o')
           
                Tsb = obj.M_(:,:,end);
                obj.draw_frame(Tsb)
        end

        function draw_frame(obj, T)

            R = T(1:3, 1:3);
            p = T(1:3, 4);
            frame_size = 0.5;

            for i = 1:3
                 p_ = p + R(:,i)*frame_size; % x,y,z axis 방향으로 그림 
                point_set = [p, p_];

                plot3(point_set(1,:), point_set(2,:), point_set(3,:)) 
                hold on;
            end

        end

    end

end
