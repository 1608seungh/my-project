
  function [s, b_collect, f_collect] = obj_ms_opt(r_target,x)
x=reshape(x,[],25);
b_collect = [];
f_collect = [];

for r= r_target 
    b=zeros(3,1);
    f=zeros(3,1);
    for x_index = x
        rx=x_index(1:3);
        mx=x_index(4:6);
        b = b + ms(r-rx,mx);
    end
    b_collect = [b_collect b]; 

 for x_index = x
        rx=x_index(1:3);
        mx=x_index(4:6);
        f = f + ms_force(r-rx,b,mx);
 end

    f_collect = [f_collect f];
end



s = -norm(f_collect);
end
