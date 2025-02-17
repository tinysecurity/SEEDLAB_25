%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters
K=1.9; % DC gain [rad/Vs]
sigma=10; % time constant reciprocal [1/s]
Kp=2.9;
I = 0;
% Right k = 1.9; Right sigma = 10; Right Kp = 2.9; Right I = 115.247491120584; Right P = 25.590311426602
% Left k = 1.8; Left sigma = 10; Left Kp = 2.9; Left I = 106.106200650665; Left P = 24.6589999228437
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
LeftT = readtable('stepResponseVelocityLeft(in).csv', 'VariableNamingRule', 'preserve');
positionData = readtable('posStepData.csv');


figure(1)
subplot(2,1,1)
plot(Voltage,'--','linewidth',2)
hold on
plot(positionData{:,3},positionData{:,5},'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
% plot(position, '--','linewidth',2)
plot(Position,'linewidth',2)

hold on
% change
plot(DesiredPosition,'--','linewidth',2)

plot(LeftT{:,3},LeftT{:,5},'linewidth',2)
hold off
legend('Simulated', 'Desired','Experimental','location' ,'southeast')
xlabel('Time (s)')
ylabel('Position')