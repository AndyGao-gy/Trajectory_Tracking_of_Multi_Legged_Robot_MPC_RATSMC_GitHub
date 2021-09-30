%% Comparison and Plot
%**************************************************************************
%******************【Author：AndyGao；Date：2021/09】**********************
%**************************************************************************
clear;    clc;

% Load raw data
load ReferenceTrajectory\ts                         ts                      % Sample interval (Time step size)
load ReferenceTrajectory\TimeSeq_rhyth              TimeSeq_rhyth           % Sampling time sequences of rhythmic movement
load ReferenceTrajectory\Zr_2W                      Zr_2W                   % Reference absolute pose of COM w.r.t. the world frame [X_r; Y_r; TH_r]

Z_2W_trial_MPC_PID = load('Comparison_Data\Z_2W_trial_MPC_PID','Z_2W_trial');
ZTilde_trial_MPC_PID = load('Comparison_Data\ZTilde_trial_MPC_PID','ZTilde_trial');
Pd_trial_MPC_PID = load('Comparison_Data\Pd_trial_MPC_PID','Pd_trial');
Pa_trial_MPC_PID = load('Comparison_Data\Pa_trial_MPC_PID','Pa_trial');
P_2W_trial_MPC_PID = load('Comparison_Data\P_2W_trial_MPC_PID','P_2W_trial');
q_a_trial_MPC_PID = load('Comparison_Data\q_a_trial_MPC_PID','q_a_trial');

Z_2W_trial_MPC_SMC = load('Comparison_Data\Z_2W_trial_MPC_SMC','Z_2W_trial');
ZTilde_trial_MPC_SMC = load('Comparison_Data\ZTilde_trial_MPC_SMC','ZTilde_trial');
Pd_trial_MPC_SMC = load('Comparison_Data\Pd_trial_MPC_SMC','Pd_trial');
Pa_trial_MPC_SMC = load('Comparison_Data\Pa_trial_MPC_SMC','Pa_trial');
P_2W_trial_MPC_SMC = load('Comparison_Data\P_2W_trial_MPC_SMC','P_2W_trial');
q_a_trial_MPC_SMC = load('Comparison_Data\q_a_trial_MPC_SMC','q_a_trial');

Z_2W_trial_MPC_NTSMC = load('Comparison_Data\Z_2W_trial_MPC_NTSMC','Z_2W_trial');
ZTilde_trial_MPC_NTSMC = load('Comparison_Data\ZTilde_trial_MPC_NTSMC','ZTilde_trial');
Pd_trial_MPC_NTSMC = load('Comparison_Data\Pd_trial_MPC_NTSMC','Pd_trial');
Pa_trial_MPC_NTSMC = load('Comparison_Data\Pa_trial_MPC_NTSMC','Pa_trial');
P_2W_trial_MPC_NTSMC = load('Comparison_Data\P_2W_trial_MPC_NTSMC','P_2W_trial');
q_a_trial_MPC_NTSMC = load('Comparison_Data\q_a_trial_MPC_NTSMC','q_a_trial');

Z_2W_trial_MPC_RATSMC = load('Comparison_Data\Z_2W_trial_MPC_RATSMC','Z_2W_trial');
ZTilde_trial_MPC_RATSMC = load('Comparison_Data\ZTilde_trial_MPC_RATSMC','ZTilde_trial');
Pd_trial_MPC_RATSMC = load('Comparison_Data\Pd_trial_MPC_RATSMC','Pd_trial');
Pa_trial_MPC_RATSMC = load('Comparison_Data\Pa_trial_MPC_RATSMC','Pa_trial');
P_2W_trial_MPC_RATSMC = load('Comparison_Data\P_2W_trial_MPC_RATSMC','P_2W_trial');
q_a_trial_MPC_RATSMC = load('Comparison_Data\q_a_trial_MPC_RATSMC','q_a_trial');

bodyHeight = 0.31;
numSamplePoints = length(TimeSeq_rhyth);
numTrial = length(ZTilde_trial_MPC_PID.ZTilde_trial);
fontSize = 10;                  % Font size in plot

%% 【Key Performance Indicators】
% 2% adjusting time
delta_stable = 0.02;
ConvergenceTime_trial_MPC_PID = zeros(3,numTrial);
ConvergenceTime_trial_MPC_SMC = zeros(3,numTrial);
ConvergenceTime_trial_MPC_NTSMC = zeros(3,numTrial);
ConvergenceTime_trial_MPC_RATSMC = zeros(3,numTrial);
% Cumulative absolute error (CAE)
CAE_trial_MPC_PID = zeros(3,numTrial);
CAE_trial_MPC_SMC = zeros(3,numTrial);
CAE_trial_MPC_NTSMC = zeros(3,numTrial);
CAE_trial_MPC_RATSMC = zeros(3,numTrial);
% Integral of square error (ISE)
ISE_trial_MPC_PID = zeros(3,numTrial);
ISE_trial_MPC_SMC = zeros(3,numTrial);
ISE_trial_MPC_NTSMC = zeros(3,numTrial);
ISE_trial_MPC_RATSMC = zeros(3,numTrial);
for trial = 1 : numTrial
    for i = 1 : 3
        %--------------------------------------------------------------------------
        % Convergence time (2% adjusting time)
        %--------------------------------------------------------------------------
        for n = numSamplePoints : -1 : 1
            if (ZTilde_trial_MPC_PID.ZTilde_trial{trial}(i,n) < -delta_stable) || (ZTilde_trial_MPC_PID.ZTilde_trial{trial}(i,n) > delta_stable)
                ConvergenceTime_trial_MPC_PID(i,trial) = n * ts;
                break;
            end
        end
        for n = numSamplePoints : -1 : 1
            if (ZTilde_trial_MPC_SMC.ZTilde_trial{trial}(i,n) < -delta_stable) || (ZTilde_trial_MPC_SMC.ZTilde_trial{trial}(i,n) > delta_stable)
                ConvergenceTime_trial_MPC_SMC(i,trial) = n * ts;
                break;
            end
        end
        for n = numSamplePoints : -1 : 1
            if (ZTilde_trial_MPC_NTSMC.ZTilde_trial{trial}(i,n) < -delta_stable) || (ZTilde_trial_MPC_NTSMC.ZTilde_trial{trial}(i,n) > delta_stable)
                ConvergenceTime_trial_MPC_NTSMC(i,trial) = n * ts;
                break;
            end
        end
        for n = numSamplePoints : -1 : 1
            if (ZTilde_trial_MPC_RATSMC.ZTilde_trial{trial}(i,n) < -delta_stable) || (ZTilde_trial_MPC_RATSMC.ZTilde_trial{trial}(i,n) > delta_stable)
                ConvergenceTime_trial_MPC_RATSMC(i,trial) = n * ts;
                break;
            end
        end
        %--------------------------------------------------------------------------
        % Cumulative absolute error (CAE)
        %--------------------------------------------------------------------------
        CAE_trial_MPC_PID(i,trial) = sum( abs( ZTilde_trial_MPC_PID.ZTilde_trial{trial}(i,:) ) );
        CAE_trial_MPC_SMC(i,trial) = sum( abs( ZTilde_trial_MPC_SMC.ZTilde_trial{trial}(i,:) ) );
        CAE_trial_MPC_NTSMC(i,trial) = sum( abs( ZTilde_trial_MPC_NTSMC.ZTilde_trial{trial}(i,:) ) );
        CAE_trial_MPC_RATSMC(i,trial) = sum( abs( ZTilde_trial_MPC_RATSMC.ZTilde_trial{trial}(i,:) ) );
        %--------------------------------------------------------------------------
        % Integral of square error (ISE)
        %--------------------------------------------------------------------------
        ISE_trial_MPC_PID(i,trial) = sum( abs( ZTilde_trial_MPC_PID.ZTilde_trial{trial}(i,:) ).^2 .* ts );
        ISE_trial_MPC_SMC(i,trial) = sum( abs( ZTilde_trial_MPC_SMC.ZTilde_trial{trial}(i,:) ).^2 .* ts );
        ISE_trial_MPC_NTSMC(i,trial) = sum( abs( ZTilde_trial_MPC_NTSMC.ZTilde_trial{trial}(i,:) ).^2 .* ts );
        ISE_trial_MPC_RATSMC(i,trial) = sum( abs( ZTilde_trial_MPC_RATSMC.ZTilde_trial{trial}(i,:) ).^2 .* ts );
    end
end

%**************************************************************************
% Mean and confidence interval
%**************************************************************************
% Mean convergence time and its confidence interval
M_ConvergenceTime_MPC_PID = zeros(3);
M_ConvergenceTime_MPC_SMC = zeros(3);
M_ConvergenceTime_MPC_NTSMC = zeros(3);
M_ConvergenceTime_MPC_RATSMC = zeros(3);
% Mean CAE and its confidence interval
M_CAE_MPC_PID = zeros(3);
M_CAE_MPC_SMC = zeros(3);
M_CAE_MPC_NTSMC = zeros(3);
M_CAE_MPC_RATSMC = zeros(3);
% Mean ISE and its confidence interval
M_ISE_MPC_PID = zeros(3);
M_ISE_MPC_SMC = zeros(3);
M_ISE_MPC_NTSMC = zeros(3);
M_ISE_MPC_RATSMC = zeros(3);
for i = 1 : 3
    %--------------------------------------------------------------------------
    % Convergence time (2% adjusting time)
    %--------------------------------------------------------------------------
    [phat,pci] = mle(ConvergenceTime_trial_MPC_PID(i,:));
    M_ConvergenceTime_MPC_PID(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(ConvergenceTime_trial_MPC_SMC(i,:));
    M_ConvergenceTime_MPC_SMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(ConvergenceTime_trial_MPC_NTSMC(i,:));
    M_ConvergenceTime_MPC_NTSMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(ConvergenceTime_trial_MPC_RATSMC(i,:));
    M_ConvergenceTime_MPC_RATSMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    %--------------------------------------------------------------------------
    % CAE
    %--------------------------------------------------------------------------
    [phat,pci] = mle(CAE_trial_MPC_PID(i,:));
    M_CAE_MPC_PID(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(CAE_trial_MPC_SMC(i,:));
    M_CAE_MPC_SMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(CAE_trial_MPC_NTSMC(i,:));
    M_CAE_MPC_NTSMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(CAE_trial_MPC_RATSMC(i,:));
    M_CAE_MPC_RATSMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    %--------------------------------------------------------------------------
    % ISE
    %--------------------------------------------------------------------------
    [phat,pci] = mle(ISE_trial_MPC_PID(i,:));
    M_ISE_MPC_PID(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(ISE_trial_MPC_SMC(i,:));
    M_ISE_MPC_SMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(ISE_trial_MPC_NTSMC(i,:));
    M_ISE_MPC_NTSMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
    [phat,pci] = mle(ISE_trial_MPC_RATSMC(i,:));
    M_ISE_MPC_RATSMC(i,:) = [phat(1), pci(1,1), pci(2,1)];
end

%**************************************************************************
% List
%**************************************************************************
Method = {'MPC_PID_x';'MPC_PID_y';'MPC_PID_z'; 'MPC_SMC_x';'MPC_SMC_y';'MPC_SMC_z'; 'MPC_NTSMC_x';'MPC_NTSMC_y';'MPC_NTSMC_z'; 'MPC_RATSMC_x';'MPC_RATSMC_y';'MPC_RATSMC_z'};
M_t = [M_ConvergenceTime_MPC_PID(:,1); M_ConvergenceTime_MPC_SMC(:,1); M_ConvergenceTime_MPC_NTSMC(:,1); M_ConvergenceTime_MPC_RATSMC(:,1)];
M_t_CI = [M_ConvergenceTime_MPC_PID(:,2:3); M_ConvergenceTime_MPC_SMC(:,2:3); M_ConvergenceTime_MPC_NTSMC(:,2:3); M_ConvergenceTime_MPC_RATSMC(:,2:3)];
M_CAE = [M_CAE_MPC_PID(:,1); M_CAE_MPC_SMC(:,1); M_CAE_MPC_NTSMC(:,1); M_CAE_MPC_RATSMC(:,1)];
M_CAE_CI = [M_CAE_MPC_PID(:,2:3); M_CAE_MPC_SMC(:,2:3); M_CAE_MPC_NTSMC(:,2:3); M_CAE_MPC_RATSMC(:,2:3)];
M_ISE = [M_ISE_MPC_PID(:,1); M_ISE_MPC_SMC(:,1); M_ISE_MPC_NTSMC(:,1); M_ISE_MPC_RATSMC(:,1)];
M_ISE_CI = [M_ISE_MPC_PID(:,2:3); M_ISE_MPC_SMC(:,2:3); M_ISE_MPC_NTSMC(:,2:3); M_ISE_MPC_RATSMC(:,2:3)];
List = table(Method, M_t, M_t_CI, M_CAE, M_CAE_CI, M_ISE, M_ISE_CI);
disp(List);



%% 【Comparison of position tracking of 6 feet】
% Cumulative absolute error (CAE)
Feet_CAE_MPC_PID = cell(6,1);
Feet_CAE_MPC_SMC = cell(6,1);
Feet_CAE_MPC_NTSMC = cell(6,1);
Feet_CAE_MPC_RATSMC = cell(6,1);
for trial = 1 : numTrial
    for legL = 1 : 6
        for i = 1 : 3
            Feet_CAE_MPC_PID{legL}(i,trial) = sum( abs( Pd_trial_MPC_PID.Pd_trial{trial}{legL}(i,:) - Pa_trial_MPC_PID.Pa_trial{trial}{legL}(i,:) ) );
            Feet_CAE_MPC_SMC{legL}(i,trial) = sum( abs( Pd_trial_MPC_SMC.Pd_trial{trial}{legL}(i,:) - Pa_trial_MPC_SMC.Pa_trial{trial}{legL}(i,:) ) );
            Feet_CAE_MPC_NTSMC{legL}(i,trial) = sum( abs( Pd_trial_MPC_NTSMC.Pd_trial{trial}{legL}(i,:) - Pa_trial_MPC_NTSMC.Pa_trial{trial}{legL}(i,:) ) );
            Feet_CAE_MPC_RATSMC{legL}(i,trial) = sum( abs( Pd_trial_MPC_RATSMC.Pd_trial{trial}{legL}(i,:) - Pa_trial_MPC_RATSMC.Pa_trial{trial}{legL}(i,:) ) );
        end
    end
end

% Plot
fig11 = figure(11);    clf;
set(gcf, 'Color','w', 'Position',[fig11.Position(1),fig11.Position(2),800,500], 'Units','pixels');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
tl = tiledlayout(6,3, 'TileSpacing','compact');  % Create block diagram layout and minimize subgraph spacing (R2019b and later versions are available)
XLabels = {'x direction','y direction','z direction'};
for legL = 1 : 6
    for i = 1 : 3
        nexttile;	% Create coordinate area object (R2019b and later versions are available)
        boxplot([Feet_CAE_MPC_PID{legL}(i,:)', Feet_CAE_MPC_SMC{legL}(i,:)', Feet_CAE_MPC_NTSMC{legL}(i,:)', Feet_CAE_MPC_RATSMC{legL}(i,:)'], 'Colors','krgb', 'Symbol','m+');
        set(gca, 'XTickLabel',[]);  % Delete the X-axis label
        set(gca, 'FontSize',fontSize-2, 'FontName','Times New Roman');
        if (i == 3)
            yyaxis right;    ylabel(['Foot',num2str(legL)], 'FontSize',fontSize, 'FontName','Times New Roman');
            set(gca, 'YTickLabel',[], 'YColor','k');
        end
        if (legL == 6)
            xlabel(XLabels{i}, 'FontSize',fontSize, 'FontName','Times New Roman');
        end
        if (legL == 1) && (i == 2)
            box_vars = findall(gca, 'Tag','Box');
            hLegend = legend(box_vars(4:-1:1), {'MPC-PID','MPC-SMC','MPC-NTSMC','MPC-RATSMC'}, 'Orientation','horizontal', 'Location','northoutside');
        end
    end
end
ylabel(tl, 'CAE (m)', 'FontSize',fontSize, 'FontName','Times New Roman');
%print(fig11,'Fig_Store\FootCAE_Polyline','-depsc','-r600');


%% 【Comparison of COM's position tracking】
fig12 = figure(12);    clf;
set(gcf, 'Color','w');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
hold on;    box on;
plot(Zr_2W(1,:),Zr_2W(2,:), 'm-','LineWidth',4);
plot(Z_2W_trial_MPC_PID.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_PID.Z_2W_trial{1}(2,:), 'k:','LineWidth',2);
plot(Z_2W_trial_MPC_SMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_SMC.Z_2W_trial{1}(2,:), 'r.-','LineWidth',2);
plot(Z_2W_trial_MPC_NTSMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_NTSMC.Z_2W_trial{1}(2,:), 'g--','LineWidth',2);
plot(Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(2,:), 'b-','LineWidth',2);
hold off;
xlabel('$X_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
ylabel('$Y_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
legend('Reference', 'MPC-PID', 'MPC-SMC', 'MPC-NTSMC', 'MPC-RATSMC', 'Box','off', 'Position',[0.63 0.67 0.26 0.22]);
% Add an arrow and an ellipse to the plot
annotation('arrow',[0.64 0.67],[0.27 0.33]);
annotation('ellipse',[0.57 0.18 0.14 0.09]);
% Partial enlarged drawing
subfig = axes('Position',[0.65 0.40 0.23 0.18]);
set(subfig, 'xlim',[1.27 1.32], 'ylim',[-0.005 0.005], 'FontSize',12, 'FontName','Times New Roman');
hold on; box on;
plot(Zr_2W(1,:),Zr_2W(2,:), 'm-','LineWidth',4);
plot(Z_2W_trial_MPC_PID.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_PID.Z_2W_trial{1}(2,:), 'k:','LineWidth',2);
plot(Z_2W_trial_MPC_SMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_SMC.Z_2W_trial{1}(2,:), 'r.-','LineWidth',2);
plot(Z_2W_trial_MPC_NTSMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_NTSMC.Z_2W_trial{1}(2,:), 'g--','LineWidth',2);
plot(Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(2,:), 'b-','LineWidth',2);
hold off;
%print(fig12,'Fig_Store\TrackingComparison_Polyline','-depsc','-r600');


%% 【Actual movement trajectories of the body and feet】
fig13 = figure(13);    clf;
set(gcf, 'Color','w');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
hold on;    box on;
plot3(Zr_2W(1,:),Zr_2W(2,:),bodyHeight*ones(1,numSamplePoints), 'm-','LineWidth',5);
plot3(Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(2,:),bodyHeight*ones(1,numSamplePoints), 'b-','LineWidth',5);
for legL = 1 : 6
    plot3(P_2W_trial_MPC_RATSMC.P_2W_trial{1}{legL}(1,:),P_2W_trial_MPC_RATSMC.P_2W_trial{1}{legL}(2,:),P_2W_trial_MPC_RATSMC.P_2W_trial{1}{legL}(3,:)+bodyHeight, 'LineWidth',2);
end
xlabel('$X_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
ylabel('$Y_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
zlabel('$Z_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
view(13.03, 43.05);	% 3D view
legend('Body-ref','Body-act','Foot1','Foot2','Foot3','Foot4','Foot5','Foot6', 'Location','northeast');
hold off;
%print(fig13,'Fig_Store\3DView_Polyline','-depsc','-r600');


fig14 = figure(14);    clf;
set(gcf, 'Color','w');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
hold on;    box on;
plot3(Zr_2W(1,:),Zr_2W(2,:),bodyHeight*ones(1,numSamplePoints), 'm-','LineWidth',5);
plot3(Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(1,:),Z_2W_trial_MPC_RATSMC.Z_2W_trial{1}(2,:),bodyHeight*ones(1,numSamplePoints), 'b-','LineWidth',5);
for legL = 1 : 6
    plot3(P_2W_trial_MPC_RATSMC.P_2W_trial{1}{legL}(1,:),P_2W_trial_MPC_RATSMC.P_2W_trial{1}{legL}(2,:),P_2W_trial_MPC_RATSMC.P_2W_trial{1}{legL}(3,:)+bodyHeight, 'LineWidth',2);
end
xlabel('$X_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
ylabel('$Y_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
zlabel('$Z_{\rm w}$ (m)', 'FontSize',fontSize, 'FontName','Times New Roman', 'Interpreter','latex');
view([0,0,1]);      % Top view
legend('Body-ref','Body-act','Foot1','Foot2','Foot3','Foot4','Foot5','Foot6', 'Position',[0.72,0.58,0.18,0.34]);
hold off;
%print(fig14,'Fig_Store\TopView_Polyline','-depsc','-r600');


%% 【Actual joint angles of the 6 legs】
fig15 = figure(15);     clf;
set(gcf, 'Color','w', 'Position',[fig15.Position(1),fig15.Position(2),800,500], 'Units','pixels');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
tl = tiledlayout(6,1, 'TileSpacing','none');  % Create block diagram layout and minimize subgraph spacing (R2019b and later versions are available)
for legL = 1 : 6
nexttile;	% Create coordinate area object (R2019b and later versions are available)
hold on;    box on;
plot(TimeSeq_rhyth,q_a_trial_MPC_RATSMC.q_a_trial{1}{legL}(1,:), 'k-','LineWidth',1);
plot(TimeSeq_rhyth,q_a_trial_MPC_RATSMC.q_a_trial{1}{legL}(2,:), 'r-','LineWidth',1);
plot(TimeSeq_rhyth,q_a_trial_MPC_RATSMC.q_a_trial{1}{legL}(3,:), 'g-','LineWidth',1);
plot(TimeSeq_rhyth,q_a_trial_MPC_RATSMC.q_a_trial{1}{legL}(4,:), 'b-','LineWidth',1);
xlim([0, TimeSeq_rhyth(end)]);
if (legL < 6)
    set(gca,'XTickLabel',[]);   % Delete the X-axis label
end
yyaxis right;    ylabel(['Leg',num2str(legL)], 'FontSize',fontSize, 'FontName','Times New Roman');
set(gca,'YTickLabel',[], 'YColor','k');
if (legL == 1)
    lgd = legend('q_1','q_2','q_3','q_4', 'Orientation','horizontal', 'Location','northoutside');
    set(lgd, 'FontSize',fontSize, 'FontName','Times New Roman');
end
hold off;
end
xlabel(tl, 'Time (s)', 'FontSize',fontSize, 'FontName','Times New Roman');
ylabel(tl, 'Joint angles (rad)', 'FontSize',fontSize, 'FontName','Times New Roman');
%print(fig15,'Fig_Store\ActualAngles_Polyline','-depsc','-r600');

