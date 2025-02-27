%% RunGphi.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters
K=0.38; % DC gain [rad/Vs]
sigma=8.5; % time constant reciprocal [1/s]

% Right k = 1.9; Right sigma = 10
% Left k = 1.65; Right sigma = 11
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('Gphi')
%
% run the simulation
%
out=sim('Gphi');
%% A Plot of the results
PhiData = readtable('phiDotStepResponse.csv', 'VariableNamingRule', 'preserve');

figure(1)
subplot(2,1,1)
plot(V_delta,'--','linewidth',2)
hold on
plot(PhiData{:,1},PhiData{:,3},'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')


subplot(2,1,2)
plot(PhiDot, '--','linewidth',2)
hold on
plot(PhiData{:,1},PhiData{:,5},'linewidth',2)
hold off
legend('Simulated','Experimental','location' ,'southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')