clc
clear
close all
filename = 'orientation-one_two-mins-from-30-seconds-clean.txt';
filename2 = '~/ROCO503x/data_for_report/orientation-one_two-mins-from-30-seconds-clean-resampled.txt';
filename3 = '~/ROCO503x/data_for_report/orientation-one_two-mins-from-30-seconds-clean-unresampled.txt';
M = csvread(filename);
time = M(:,1);
time = time - 31.036;
a1 = M(:,2);
a2 = M(:,3);
a3 = M(:,4);
g1 = M(:,5);
g2 = M(:,6);
g3 = M(:,7);

[a1, yt] = resample(a1,time);
a2 = resample(a2,time);
a3 = resample(a3,time);

g1 = resample(g1,time);
g2 = resample(g2,time);
g3 = resample(g3,time);

M2 = [yt,a1,a2,a3,g1,g2,g3]
csvwrite('~/ROCO503x/data_for_report/orientation-one-resampled-full.txt',M2)

%[time,a1,a2,a3,g1,g2,g3] = textread("orientation-one_two-mins-from-30-seconds-clean.txt", "%f%f%f%f%f%f%f", "delimiter", ", ", "endofline", "\n");

%Load File
%#file = 'e-neu.wav';
%[y,Fs,bits] = wavread(file);
x = a1;
tx = time;
[y, ty] = resample(x, tx);
ty
y
M2 = [ty, y]
M3 = [time,a1]
csvwrite(filename2,M2)
csvwrite(filename3,M3)

tz = sum(ty) - ty(1)
tw = sum(ty) - 119.9880
tn = tz - tw
tm = tn / length(ty)


Fs = tm;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 120;             % Length of signal
t = (0:L-1)*T;        % Time vector

Y = fft(y)

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

Y = fft(S);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

plot(f,P1) 
title('Single-Sided Amplitude Spectrum of S(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')