
close all

mV = data(:,2);
PWM_in_steps = data(:,1);
PWM = PWM_in_steps./885;
rpm = data(:,3);
          
omega = rpm .*0.1047;
tau = (mV / 1000) * 2


C = [tau omega]\PWM;

tau_calculated = (1/C(1)) * (PWM - omega*C(2))

figure()
scatter(tau_calculated, tau, 'filled')
ylim([-1,5])
xlim([-1,5])
grid on

hold on

plot([-1, 5], [-1,5])








