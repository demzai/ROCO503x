%Fourier Transform of Sound File

[time,a1,a2,a3,g1,g2,g3] = textread("orientation-one_two-mins-from-30-seconds-clean.txt", "%f%f%f%f%f%f%f", "delimiter", ", ", "endofline", "\n");
time = time - 31.036;
%Load File
%#file = 'e-neu.wav';
%[y,Fs,bits] = wavread(file);
f = 150;
Fs = 150;
y = a1;
y1 = resample(y,153,150);
find(isnan(y))
find(isinf(y))
Nsamps = length(y);
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

%Plot Sound File in Frequency Domain
figure
plot(f, y_fft);
xlim([0 100])
xlabel('Frequency (Hz)')
ylabel('Amplitude')
title('Frequency Response of Tuning Fork A4')
