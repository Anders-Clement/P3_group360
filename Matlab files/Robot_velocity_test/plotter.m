close all

table2plot = velocities_base;

array2plot = table2array(table2plot(1,:));

t = linspace(1, size(array2plot,2), size(array2plot,2));
plot(t,array2plot);
grid on
xlabel('Time in steps')
ylabel('velocity [mm/s]')
hold on
x = 0:size(t,2);
y = linspace(250,250,size(x,2));
plot(x,y);
xlim([0 size(t,2)]);