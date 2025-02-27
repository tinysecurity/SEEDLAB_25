%% RunGrho.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters
K=0.07; % DC gain [rad/Vs]
sigma=11; % time constant reciprocal [1/s]

% Right k = 1.9; Right sigma = 10
% Left k = 1.65; Right sigma = 11
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('Grho')
%
% run the simulation
%
out=sim('Grho');
%% A Plot of the results
RhoData = readtable('rhoDotStepResponse.csv', 'VariableNamingRule', 'preserve');

figure(1)
subplot(2,1,1)
plot(V_bar,'--','linewidth',2)
hold on
plot(RhoData{:,1},RhoData{:,2},'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('V_Bar (V)')


subplot(2,1,2)
plot(RhoDot, '--','linewidth',2)
hold on
plot(RhoData{:,1},RhoData{:,4},'linewidth',2)
hold off
legend('Simulated','Experimental','location' ,'southeast')
xlabel('Time (s)')
ylabel('Rho')