% Author: Se Hwan
% AME 50572: Intro to CFD
% Plotting order of accuracy

clc;clf; clear all
%% data
r=2;
h = [8 4 2 1];
pressureDE_L1 = [0.0022 0.3655e-3 0.6991e-4 0.1498e-4 0.3434e-5];
pressureDE_L2 = [0.0030 0.5776e-3 0.1140e-3 0.2372e-4 0.5350e-5];
pressureDE_Linf = [0.0072 0.0017 0.6016e-3 0.2181e-3 0.8203e-4];

uDE_L1 = [0.0002  0.0637e-3 0.1778e-4 0.0457e-4 0.10793e-5];
uDE_L2 = [0.0004 0.0926e-3 0.0245e-3 0.0603e-4 0.1394e-5];
uDE_Linf = [0.0011 0.0003 0.0568e-3 0.0133e-3 0.0303e-4];

vDE_L1 = [0.0001  0.0128e-3 0.0351e-4 0.0089e-4 0.0215e-5];
vDE_L2 = [0.0001 0.0160e-3 0.0042e-3 0.0104e-4 0.0247e-5];
vDE_Linf = [0.0002  0.0001 0.0162e-3 0.0040e-3 0.0098e-4];



for i=1:length(h)
    p_L1(i) = log(pressureDE_L1(i)/pressureDE_L1(i+1))/log(r);
    p_L2(i) = log(pressureDE_L2(i)/pressureDE_L2(i+1))/log(r);
    p_Linf(i) = log(pressureDE_Linf(i)/pressureDE_Linf(i+1))/log(r);
    u_L1(i) = log(uDE_L1(i)/uDE_L1(i+1))/log(r);
    u_L2(i) = log(uDE_L2(i)/uDE_L2(i+1))/log(r);
    u_Linf(i) = log(uDE_Linf(i)/uDE_Linf(i+1))/log(r);
    v_L1(i) = log(vDE_L1(i)/vDE_L1(i+1))/log(r);
    v_L2(i) = log(vDE_L2(i)/vDE_L2(i+1))/log(r);
    v_Linf(i) = log(vDE_Linf(i)/vDE_Linf(i+1))/log(r);
end

%% plotting
hold off;

figure(1)
semilogx(h,p_L1,'r.-',h,p_L2,'r.--',h,p_Linf,'ro-')
hold
semilogx(h,u_L1,'b.-',h,u_L2,'b.--',h,u_Linf,'bo-')
semilogx(h,v_L1,'g.-',h,v_L2,'g.--',h,v_Linf,'go-')
xlabel('Iteration')
ylabel('Iterative Residuals')
legend('P: L1','P: L2','P: L_{\infty}','u: L1','u: L2','u: L_{\infty}','v: L1','v: L2','v: L_{\infty}')
axis([1,18,0,3])
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times New Roman';