clc
clear
close all
filename = 'orientation-one_two-mins-from-30-seconds-clean.txt';
filename2 = '~/ROCO503x/data_for_report/orientation-one_two-mins-from-30-seconds-clean-resampled.txt';
filename3 = '~/ROCO503x/data_for_report/orientation-one_two-mins-from-30-seconds-clean-unresampled.txt';
M = csvread(filename);
a1 = M(:,2)
time = M(:,1)
%[time,a1,a2,a3,g1,g2,g3] = textread("orientation-one_two-mins-from-30-seconds-clean.txt", "%f%f%f%f%f%f%f", "delimiter", ", ", "endofline", "\n");
time = time - 31.036;
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