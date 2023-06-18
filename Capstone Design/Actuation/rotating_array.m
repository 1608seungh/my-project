while(1)
    clear a;
    clear all; clc;
    
    a = arduino();    
    s = servo(a,"D8", 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6);
    
    for i = 1:480 % counter clockwise
        writePosition(s, i/480);
    end
    
    for j = 1:480 % clockwise
        writePosition(s, (480-j)/480);
    end
end

% 
% writePosition(s, 90/180);
