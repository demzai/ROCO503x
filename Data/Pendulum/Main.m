close all
clear
clc
PendulumCodeNorm
PendulumCodeComp
pcnR = PCN(:,1:7);
pcnF = PCN(:,8:14);
pcnRD = PCN(:,15:34);
pcnFD = PCN(:,35:54);
pccRD = PCC(:,15:34);
pccFD = PCC(:,35:54);


figure(1);
% plot(pcnR(:,1),pcnR(:,2),pcnF(:,1),pcnF(:,2));

% 0 = Acceleration
% 1 = Velocity
% 2 = Position
% 3 = Gyro
% 4 = Angle

dataset = 2;
axis1 = 1;
data1 = 1+axis1+dataset*3;
axis2 = 2;
data2 = 1+axis2+dataset*3;
% plot3(pcnRD(1:4891,1),pcnRD(1:4891,data1),pcnRD(1:4891,data2));
% hold all;
plot3(pcnFD(1:4891,1),pcnFD(1:4891,data1),pcnFD(1:4891,data2), ...
        pccFD(1:4891,1),pccFD(1:4891,data1),pccFD(1:4891,data2));
legend("FDR", "FDR+Comp");








