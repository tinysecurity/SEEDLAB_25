%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: Grho.slx
%
%% Define motor parameters
K=1.65; % DC gain [rad/Vs]
sigma=11; % time constant reciprocal [1/s]
Kp=2.8;
% Right k = 1.9; Right sigma = 10; Right Kp = 2.9
% Left k = 1.65; Left sigma = 11; Left Kp = 2.8
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motor_control')
%
% run the simulation
%
out=sim('motor_control');
%% A Plot of the results
RightT = readtable('stepResponseVelocityRight(in).csv', 'VariableNamingRule', 'preserve');
LeftT = readtable('StepResponseNewLeft.csv', 'VariableNamingRule', 'preserve');


figure(1)
subplot(2,1,1)
plot(Voltage,'--','linewidth',2)
hold on
plot(LeftT{:,1},LeftT{:,2},'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
% plot(Velocity, '--','linewidth',2)
plot(Velocity,'linewidth',2)

hold on
% change
plot(DesiredVelocity,'--','linewidth',2)

plot(LeftT{:,1},LeftT{:,3},'linewidth',2)
hold off
legend('Simulated', 'Desired','Experimental','location' ,'southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')