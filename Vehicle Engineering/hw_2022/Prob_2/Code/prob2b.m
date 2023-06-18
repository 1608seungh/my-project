load vehicle_lat_pos;

y=vehicle_lat_pos;
y_filt = zeros(size(y));

%%%%%
a_filt = 0.01; % 0 ~ 1, try , 0.01, 0.1, 0.5, 0.9, 1
%%%%%

y_filt(1)=y(1);

for k=2:length(y)
    y_filt(k) = (1-a_filt)*y_filt(k-1) + a_filt*y(k);
end

figure(22)
clf
set(gcf, 'position', [200 200 600 300]);

plot(y/100);
hold on
plot(y_filt/100);
grid on
xlabel('driving distance (m)')
ylabel('lateral deviation (m)');
title('deviation from the lane center');
legend('measured', 'filtered');
mytext=['a filt = ', num2str(a_filt)];
text(800, 1, mytext);