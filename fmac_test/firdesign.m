% Low pass FIR filter design
close all
clear
pkg load signal
Fs=48000; % Sample rate used by the board
Fpass=1000; % Set the cut-off to 8kHz
Fstop=Fpass*1.5;
Wp=Fpass/(Fs/2);
Order=64;
b=fir1(Order,Wp);
[h,w]=freqz(b); %,1,1024,Fs);
mag=20*log10(abs(h));
semilogx(w*(Fs/(2*pi)), mag)
bq=floor(b*32767/max(b));
t=0:1/Fs:100e-3;
vin=32767*sin(2*pi*8000*t);
y=filtfilt(b,1,vin);
ScaleFactor=Fpass/max(b);
%plot(b*ScaleFactor)
figure()
plot(b)

