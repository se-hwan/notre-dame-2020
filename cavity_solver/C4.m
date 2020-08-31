% Author: Se Hwan
% AME 50572: Intro to CFD
% Plotting effect of C4

clc;clf; clear all
%% data
c4small = importdata('difference001.mat');
c4med = importdata('difference003.mat');
c4big = importdata("difference005.mat");
n = 65;
x = linspace(0,0.05,n);
errorSmall = c4small(cast(n/2,'int8'),:,1);
errorMed = c4med(cast(n/2,'int8'),:,1);
errorBig = c4big(cast(n/2,'int8'),:,1);


%% plotting
hold off;

figure(1)
plot(x,errorSmall,'ro-',x,errorMed,'bo-',x,errorBig,'go-')
xlabel('x (m)')
ylabel('Discretization error')
legend('C^{(4)} = 0.01','C^{(4)} = 0.03','C^{(4)} = 0.05')
axis([0 0.01 0 2.1*10^-4])
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times New Roman';