%% 
controller = 1; % 1: SFC
                % 2: LQR
                % 3: SMC

pos_ref = 0;
%%

figure(1)
xline(0)
hold on
yline(0)
ylabel('Beam Angle (deg)','fontsize',16,'interpreter','latex')
xlabel('Ball Position (m)','fontsize',16,'interpreter','latex')

if(controller == 1)
    title("SFC with integral action",'fontsize',16,'interpreter','latex')
    Files=dir('SFC_data/*');
elseif(controller == 2)
    title("LQR with integral action",'fontsize',16,'interpreter','latex')
    Files=dir('LQR_data/*');
elseif(controller == 3)
    title("Sliding Mode Control",'fontsize',16,'interpreter','latex')
    Files=dir('SMC_data/*');
end

for k=1:length(Files)
   FileName = Files(k).name;

    data = readtable('LQR_data/scope_37.csv');
    data = data(4:end,:);
    
    ballPos = data.Var3;
    beamAngle = data.Var4;

    % do conversion to real units
    
    ballPos = (16.838518 .* ballPos + 2.763498)/100;
    ballPos = ballPos - pos_ref;
    beamAngle = 1.040051 .* beamAngle + 0.696110;
    
    % some filtering
    
    ballPos = movmean(ballPos,30);
    beamAngle = movmean(beamAngle,30);
    
    plot(ballPos,beamAngle)
end
