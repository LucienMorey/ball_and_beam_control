%%
theta_dot = diff(beamAngle)/ts;

Nf = 50; 
Fpass = 0.0001; 
Fstop = 150;

d = designfilt('differentiatorfir','FilterOrder',Nf, ...
    'PassbandFrequency',Fpass,'StopbandFrequency',Fstop, ...
    'SampleRate',Fs);

dt = ts;

vdrift = filter(d,beamAngle)/dt;
delay = mean(grpdelay(d))
tt = data.Var1(1:end-delay);
vd = vdrift;
vd(1:delay) = [];

tt(1:delay) = [];
vd(1:delay) = [];

[pkp,lcp] = findpeaks(beamAngle);
zcp = zeros(size(lcp));

[pkm,lcm] = findpeaks(-beamAngle);
zcm = zeros(size(lcm));

subplot(2,1,1)
plot(data.Var1,beamAngle,data.Var1([lcp lcm]),[pkp -pkm],'or')
xlabel('Time (s)')
ylabel('Displacement (cm)')
grid

subplot(2,1,2)
plot(tt,vd,data.Var1([lcp lcm]),[zcp zcm],'or')
xlabel('Time (s)')
ylabel('Speed (cm/s)')
grid

table_data = iddata(theta_dot,inputVoltage(1:end-1,:),ts);

%%
% syms z
% func =  (-0.0001156*z^-1 + 0.0001712*z^-2) / (1 - 1.905*z^-1 + 0.905*z^-2)
% iztrans(func)