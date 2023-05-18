%% 
controller = 2; % 1: SFC
                % 2: LQR
                % 3: SMC

pos_ref = 0.0;
averaging = 50;
%%

figure(1)
xline(0)
hold on
yline(0)
ylabel('Beam Angle (deg)','fontsize',16,'interpreter','latex')
xlabel('Ball Position (m)','fontsize',16,'interpreter','latex')

if(controller == 1)
    title("SFC with integral action",'fontsize',16,'interpreter','latex')
    Files=dir('SFC_data/');
elseif(controller == 2)
    title("LQR with integral action",'fontsize',16,'interpreter','latex')
    Files=dir('LQR_data/');
elseif(controller == 3)
    title("Sliding Mode Control",'fontsize',16,'interpreter','latex')
    Files=dir('SMC_data/');
end

for k=1:(size(Files,1)-2)
    FileName = Files(2+k,:).name;

    data = readtable(horzcat(Files(k,:).folder,'/',FileName));
    data = data(4:end,:);
    
    ballPos = data.Var3;
    beamAngle = data.Var4;

    % do conversion to real units
    
    ballPos = (16.838518 .* ballPos + 2.763498)/100;
    ballPos = ballPos - pos_ref;
    beamAngle = 1.040051 .* beamAngle + 0.696110;
    
    % some filtering
    
    ballPos = movmean(ballPos,averaging);
    beamAngle = movmean(beamAngle,averaging);
    
    plot(ballPos,beamAngle)
end

%% Plot time response just for 1st dataset
figure(2)
ts = data.Var1(2) - data.Var1(1);
time = [0:ts:ts*size(data.Var1,1)-ts]'
subplot(2,1,1)
plot(time,ballPos)
hold on
plot(time,beamAngle)
if(controller == 1)
    title("SFC with integral action",'fontsize',16,'interpreter','latex');
elseif(controller == 2)
    title("LQR with integral action",'fontsize',16,'interpreter','latex')
elseif(controller == 3)
    title("Sliding Mode Control",'fontsize',16,'interpreter','latex')
end
subplot(2,1,2)
plot(time,data.Var2)
