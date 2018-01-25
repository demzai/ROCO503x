%Fourier Transform of Sound File
filename = 'orientation-one_two-mins-from-30-seconds-clean.txt';
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
tz = sum(ty) - ty(1)
tw = sum(ty) - 119.9880
tn = tz - tw
tm = tn / length(ty)


tx = mean(ty)

find(isnan(y))
find(isinf(y))
Nsamps = length(y);
Fs = 1/(tm)
t = (1/Fs)*(1:Nsamps);          %Prepare time data for plot
%t = time;
%Do Fourier Transform
y_fft1 = abs(fft(y));            %Retain Magnitude
y_fft = y_fft1(1:Nsamps/2);      %Discard Half of Points
%f = Fs*(0:Nsamps/2-1)/Nsamps;   %Prepare freq data for plot

%Plot Sound File in Time Domain
figure
plot(t, y);
xlabel('Time (s)')
ylabel('Amplitude')
title('fft action')


L = 60*2*1000
L = 119.9880 * 1000
f = Fs*(0:(L/2))/L;
P2 = abs(y_fft/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);


%Plot Sound File in Frequency Domain
L
figure
plot(f, y_fft);
xlim([0 100])
xlabel('Frequency (Hz)')
ylabel('Amplitude')
title('Frequency Response of Tuning Fork A4')
