clc
clear

path = '~/ROCO503x/data_for_report/task5/';
file = 'try2';


f1 = 'orientation-';
f3 = '-clean';
f4 = '.txt';
f5 = '-resampled-full.txt';
upndown = 'upndown-clean';

%filename = sprintf('%s%s%s%s%s', path, f1, file, f3, f4)
filename = sprintf('%s%s%s%s', path, file, f3, '.txt')

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

%outputfile = sprintf('%s%s%s%s%s%d%s', path, f1, file, f3, '-', tm, '-resampled.txt')
outputfile = sprintf('%s%s%d%s', path, 'try1-resampled-', tm, '.txt')
%csvwrite(outputfile,M2)