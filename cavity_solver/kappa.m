% Author: Se Hwan
% AME 50572: Intro to CFD
% Plotting convergence history

clc;clf; clear all
%% data
kappa01 = importdata('history_SGS_65.dat');
kappa05 = importdata('history_kappa_05.dat');

iter_kappa01 = kappa01.data(:,1);
p_kappa01 = kappa01.data(:,3);
u_kappa01 = kappa01.data(:,4);
v_kappa01 = kappa01.data(:,5);

iter_kappa05 = kappa05.data(:,1);
p_kappa05 = kappa05.data(:,3);
u_kappa05 = kappa05.data(:,4);
v_kappa05 = kappa05.data(:,5);

%% plotting
hold off;

figure(1)
semilogy(iter_kappa01,p_kappa01,'r.-',iter_kappa01,u_kappa01,'b.-',iter_kappa01,v_kappa01,'g.-')
hold
semilogy(iter_kappa05,p_kappa05,'r--',iter_kappa05,u_kappa05,'b--',iter_kappa05,v_kappa05,'g--')
xlabel('Iteration')
ylabel('Iterative Residuals')
legend('P: \kappa=0.1','u: \kappa=0.1','v: \kappa=0.1','P: \kappa=0.5','u: \kappa=0.5','v: \kappa=0.5')
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times New Roman';