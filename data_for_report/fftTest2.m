clc
clear
close all


path = '~/ROCO503x/data_for_report/';
file = 'one';


f1 = 'orientation-';
f3 = '-clean';
f4 = '.txt';
f5 = '-resampled-full.txt';

filename = sprintf('%s%s%s%s%s', path, f1, file, f3, f4)


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

M2 = [yt,a1,a2,a3,g1,g2,g3];


tz = sum(yt) - yt(1);
%tw = sum(yt) - 119.9880;
tw = sum(yt) - yt(end);
tn = tz - tw;
tm = tn / length(yt)

outputfile = sprintf('%s%s%s%s%s%d%s', path, f1, file, f3, '-', tm, f4)
csvwrite('~/ROCO503x/data_for_report/orientation-two-resampled-full.txt',M2)









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