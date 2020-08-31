% Author: Se Hwan
% AME 50572: Intro to CFD
% Plotting convergence history

clc;clf; clear all
%% data
SGS_65 = importdata('history_SGS_65.dat');
PJ_65 = importdata("history_PJ_65.dat");

iter_SGS_65 = SGS_65.data(:,1);
p_SGS_65 = SGS_65.data(:,3);
u_SGS_65 = SGS_65.data(:,4);
v_SGS_65 = SGS_65.data(:,5);

iter_PJ_65 = PJ_65.data(:,1);
p_PJ_65 = PJ_65.data(:,3);
u_PJ_65 = PJ_65.data(:,4);
v_PJ_65 = PJ_65.data(:,5);

%% plotting
hold off;

figure(1)
semilogy(iter_SGS_65,p_SGS_65,'r.-',iter_SGS_65,u_SGS_65,'b.-',iter_SGS_65,v_SGS_65,'g.-')
hold
semilogy(iter_PJ_65,p_PJ_65,'r--',iter_PJ_65,u_PJ_65,'b--',iter_PJ_65,v_PJ_65,'g--')
xlabel('Iteration')
ylabel('Iterative Residuals')
legend('P: SGS 65\times65','u: SGS 65\times65','v: SGS 65\times65','P: PJ 65\times65','u: PJ 65\times65','v: PJ 65\times65')
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times New Roman';