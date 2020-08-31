% Author: Se Hwan
% AME 50572: Intro to CFD
% Plotting DE vs. h for manufactured solution

clc;clf;
%% data
h = [16 8 4 2 1];
pressureDE_L1 = [0.0022 0.3655e-3 0.6991e-4 0.1498e-4 0.3434e-5];
pressureDE_L2 = [0.0030 0.5776e-3 0.1140e-3 0.2372e-4 0.5350e-5];
pressureDE_Linf = [0.0072 0.0017 0.6016e-3 0.2181e-3 0.8203e-4];

uDE_L1 = [0.0002  0.0637e-3 0.1778e-4 0.0457e-4 0.10793e-5];
uDE_L2 = [0.0004 0.0926e-3 0.0245e-3 0.0603e-4 0.1394e-5];
uDE_Linf = [0.0011 0.0003 0.0568e-3 0.0133e-3 0.0303e-4];

vDE_L1 = [0.0001  0.0128e-3 0.0351e-4 0.0089e-4 0.0215e-5];
vDE_L2 = [0.0001 0.0160e-3 0.0042e-3 0.0104e-4 0.0247e-5];
vDE_Linf = [0.0002  0.0001 0.0162e-3 0.0040e-3 0.0098e-4];

%% plotting
hold on;
figure(1)
loglog(h,pressureDE_L1,'ro-',h,pressureDE_L2,'r.-',h,pressureDE_Linf,'r*--',h,vDE_L1,'go-',h,vDE_L2,'g.-',h,vDE_Linf,'g*--',h,uDE_L1,'bo-',h,uDE_L2,'b.-',h,uDE_Linf,'b*--')
xlabel('h')
ylabel('DE Norms')
legend('P: L1','P: L2','P: L_{\infty}','v: L1','v: L2','v: L_{\infty}','u: L1','u: L2','u: L_{\infty}')
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times New Roman';