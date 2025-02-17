%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters
K=1.8; % DC gain [rad/Vs]
sigma=10; % time constant reciprocal [1/s]

% Right k = 1.9; Right sigma = 10
% Left k = 1.8; Right sigma = 10
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motorsim')
%
% run the simulation
%
out=sim('motorsim');
%% A Plot of the results

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
plot(Velocity, '--','linewidth',2)
hold on
plot(LeftT{:,1},LeftT{:,3},'linewidth',2)
hold off
legend('Simulated','Experimental','location' ,'southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')