%% MPC-SMC
%**************************************************************************
%******************【Author：AndyGao；Date：2021/09】**********************
%**************************************************************************
clear;    clc;
FUNCSET = FUNCTIONSET(); % Instantiation

%% 【Structural dimensions and nominal configurations of the robot WelCH】
global Measure Alpha Theta_0 beta_0 phi_0
[Measure,Alpha] = FUNCSET.ROBOTMEASURES();	% Structural dimensions
bodyHeight = Measure(3) + Measure(4);       % Body height ( L3+L4 )
Theta_0 = zeros(6,4);	% Nominal configuration (Initial values of joint angles (Unit: rad))
beta_0 = deg2rad(0);	% Initial relative angle of the sucker (Unit: rad)
phi_0 = deg2rad(0);     % Initial pitch angle of the body platform (Unit: rad)

%% 【Load variables and parameters】
load ReferenceTrajectory\SwiSta                     SwiSta                  % Flag of swing phase and stance phase (1 for swing; -1 for stance)
load ReferenceTrajectory\ts                         ts                      % Sample interval (Time step size)
load ReferenceTrajectory\Nswing                     Nswing                  % Interpolation points of a single swing period【Must be consistent with vrep_main.m】
load ReferenceTrajectory\Nstride                    Nstride                 % Interpolation points of a single stride period
load ReferenceTrajectory\Nrhyth                     Nrhyth                  % Interpolation points of rhythmic movement
load ReferenceTrajectory\TimeSeq_rhyth              TimeSeq_rhyth           % Sampling time sequences of rhythmic movement
load ReferenceTrajectory\GammaFoot_horiz_rhyth      GammaFoot_horiz_rhyth   % Foot interpolation in horizontal direction during rhythmic movement
load ReferenceTrajectory\GammaFoot_verti_rhyth      GammaFoot_verti_rhyth   % Foot interpolation in vertical direction during rhythmic movement
load ReferenceTrajectory\GammaBody_rhyth            GammaBody_rhyth         % Body interpolation during rhythmic movement
load ReferenceTrajectory\DGammaBody_rhyth           DGammaBody_rhyth        % Derivative of the body interpolation during rhythmic movement
load ReferenceTrajectory\Ur                         Ur                      % Reference control input [Sx; Sy; Sz]
load ReferenceTrajectory\Zr_2W                      Zr_2W                   % Reference absolute pose of COM w.r.t. the world frame [X_r; Y_r; TH_r]
load ReferenceTrajectory\Pr_2B                      Pr_2B                   % Reference relative position of FOOT w.r.t. the instantaneous body frame

TimeSeq = TimeSeq_rhyth;                % Sampling time sequences


%% 【Parameter setting】
%**************************************************************************
% Parameters of MPC
%**************************************************************************
Nx = 3;                                 % Number of states
Nu = 3;                                 % Number of inputs
Np = 4;                                 % Preview horizon
Nc = 2;                                 % Control horizon
Q = diag([200,200,200]);                % State weight in quadratic programming
R = 100 * eye(3);                       % Input weight in quadratic programming
QTilde = kron(eye(Np), Q);              % Weight of predictive state
RTilde = kron(eye(Nc), R);              % Weight of predictive control increment
OnesTilde = kron(ones(Nc,1), eye(Nu));	% Generalized one vector
EyeTilde = kron(tril(ones(Nc)), eye(Nu));% Lower triangular matrix
% Upper and lower limits of variables
ZTilde_min = -[0.03; 0.03; 0.5];                        ZTilde_max = -ZTilde_min;
UTilde_min = -[0.04; 0.04; 0.2];                        UTilde_max = -UTilde_min;
DeltaUTilde_min = -[0.01; 0.01; 0.1];                   DeltaUTilde_max = -DeltaUTilde_min;
ZHat_min = kron(ones(Np,1), ZTilde_min);                ZHat_max = kron(ones(Np,1), ZTilde_max);
UHat_min = kron(ones(Nc,1), UTilde_min);                UHat_max = kron(ones(Nc,1), UTilde_max);
DeltaUHat_min = kron(ones(Nc,1), DeltaUTilde_min);      DeltaUHat_max = kron(ones(Nc,1), DeltaUTilde_max);

%**************************************************************************
% Parameters of SMC
%**************************************************************************
DoF = 3;                                % Degrees of freedom of the foot translation
Kp = 5000 * eye(DoF);
Ki = 1000 * eye(DoF);
sigma = 0.2;


%% 【Variables & Initial values】
%**************************************************************************
% Variables corresponding to MPC
%**************************************************************************
Um = zeros(Nu,Nrhyth);                  % Candidate body's stride by MPC [Sx_m; Sy_m; Sz_m]
Zd_2B = zeros(Nx,Nrhyth);               % Desired relative pose of COM w.r.t. the initial body frame by MPC [xB_d; yB_d; thB_d]
Z_2B = zeros(Nx,Nrhyth);                % Actual relative pose of COM w.r.t. the initial body frame by MPC [xB; yB; thB]
Z_2W = zeros(Nx,Nrhyth);                % Actual absolute pose of COM w.r.t. the world frame [X; Y; TH]
Pd_2B = cell(6,1);                      % Desired relative position of FOOT w.r.t. the instantaneous body frame
P_2B = cell(6,1);                       % Actual relative position of FOOT w.r.t. the instantaneous body frame
P_2W = cell(6,1);                       % Actual absolute position of FOOT w.r.t. the world frame
q_a = cell(6,1);                        % Actual joint angles [q1;q2;q3;q4]
ZTilde = zeros(Nx,Nrhyth);              % Error of COM's pose
UTilde = zeros(Nu,Nrhyth);              % Error of body stride
DeltaUTilde = zeros(Nu,Nrhyth);         % Increment of stride error
A_k = cell(Np,1);                       % System matrix in the preview horizon
B_k = cell(Np,1);                       % Control matrix in the preview horizon
G_k = cell(Np,1);                       % Discrete system matrix in the preview horizon
H_k = cell(Np,1);                       % Discrete control matrix in the preview horizon
GBar = cell(Np,1);
HBar = cell(Np,1);
HHat = cell(Np,Nc);

%**************************************************************************
% Variables corresponding to SMC
%**************************************************************************
Pd = cell(6,1);                         % Desired relative position of FOOT w.r.t. the instantaneous body frame
DPd = cell(6,1);                        % Desired relative velocity of FOOT w.r.t. the instantaneous body frame
Pa = cell(6,1);                         % Actual relative position of FOOT w.r.t. the instantaneous body frame
Dq_ideal = cell(6,1);                   % Ideal (unconstrained) angular velocity
q_ideal = cell(6,1);                    % Ideal (unconstrained) joint angles
q_const = cell(6,1);                    % Constrained joint angles
e = cell(6,1);                          % Error of foot position
e_integ = cell(6,1);                    % Integral of foot position error
e_deriv = cell(6,1);                    % Derivative of foot position error
De_deriv = cell(6,1);                   % Second-order derivative of foot position error
s = cell(6,1);                          % Sliding surface
Ds = cell(6,1);                         % Derivative of sliding surface
U_SMC = cell(6,1);                      % Limb-level controller

%**************************************************************************
% Initial values
%**************************************************************************
for legL = 1 : 6
    %--------------------------------------------------------------------------
    % Initial values corresponding to MPC
    %--------------------------------------------------------------------------
    Um(:,1) = Ur(:,1);
    Zd_2B(:,1) = zeros(3,1);
    Z_2B(:,1) = zeros(3,1);
    Z_2W(:,1) = [0.10;0.18;-0.25];
    Pd_2B{legL}(:,1) = Pr_2B{legL}(:,1);
    P_2B{legL}(:,1) = [Pr_2B{legL}(1,1)-0.02*legL; Pr_2B{legL}(2,1)-0.02*legL; Pr_2B{legL}(3,1)];
    P_2W{legL}(1:2,1) = Z_2W(1:2,1) + rot2(Z_2W(3,1)) * P_2B{legL}(1:2,1);
    P_2W{legL}(3,1) = P_2B{legL}(3,1);
    q_a{legL}(:,1) = zeros(4,1);
    ZTilde(:,1) = Z_2W(1:3,1) - Zr_2W(1:3,1);
    UTilde(:,1) = [0;0;0];
    DeltaUTilde(:,1) = [0;0;0];
    
    %--------------------------------------------------------------------------
    % Initial values corresponding to SMC
    %--------------------------------------------------------------------------
    Pd{legL}(:,1) = Pd_2B{legL}(:,1);
    DPd{legL}(:,1) = zeros(DoF,1);
    Dq_ideal{legL}(:,1) = zeros(DoF,1);
    q_ideal{legL}(:,1) = zeros(4,1);
    q_const{legL}(:,1) = q_ideal{legL}(:,1);
    Pa{legL}(:,1) = P_2B{legL}(:,1);
    e{legL}(:,1) = Pd{legL}(:,1) - Pa{legL}(:,1);
    e_integ{legL}(:,1) = zeros(DoF,1);
    e_deriv{legL}(:,1) = zeros(DoF,1);
    De_deriv{legL}(:,1) = zeros(DoF,1);
    s{legL}(:,1) = Kp * e{legL}(:,1) + Ki * e_integ{legL}(:,1) + e_deriv{legL}(:,1);
    Ds{legL}(:,1) = zeros(DoF,1);
end


%% 【Hybrid Control】
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
% Start random trials
numTrial = 30;
Zr_2W_trial = cell(1,numTrial);
Z_2W_trial = cell(1,numTrial);
ZTilde_trial = cell(1,numTrial);
Pd_trial = cell(1,numTrial);
Pa_trial = cell(1,numTrial);
P_2W_trial = cell(1,numTrial);
q_a_trial = cell(1,numTrial);
for trial = 1 : numTrial
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

% Hybrid Control
Disturbance = trial*0.005*[cos(2*TimeSeq); sin(3*TimeSeq); cos(TimeSeq)]; % Lumped disturbance
for k = 2 : Nrhyth
    nst = floor((k-1) / Nstride) + 1;                       % Number of stride period corresponding to the current moment
    nsw = floor(((k-1) - Nstride * (nst-1)) / Nswing) + 1;	% Number of swing period corresponding to the current moment
    for legL = 1 : 6
        alphaL = Alpha(legL); % Limb's azimuth angle
        %--------------------------------------------------------------------------
        % MPC
        %--------------------------------------------------------------------------
        for i = 1 : Np
            A_k{i} = [ 0,   0,   -DGammaBody_rhyth(k)*(sin(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(1,k) + cos(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(2,k));
                       0,   0,    DGammaBody_rhyth(k)*(cos(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(1,k) - sin(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(2,k));
                       0,   0,    0];
            B_k{i} = [ DGammaBody_rhyth(k)*cos(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k)),  -DGammaBody_rhyth(k)*sin(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k)),   DGammaBody_rhyth(k)*GammaBody_rhyth(k)*(sin(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(1,k) + cos(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(2,k));
                       DGammaBody_rhyth(k)*sin(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k)),   DGammaBody_rhyth(k)*cos(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k)),  -DGammaBody_rhyth(k)*GammaBody_rhyth(k)*(cos(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(1,k) - sin(Zr_2W(3,k)-GammaBody_rhyth(k)*Ur(3,k))*Ur(2,k));
                       0,                                                                                0,                                                                                DGammaBody_rhyth(k)];
            G_k{i} = eye(3) + ts .* A_k{i};
            H_k{i} = ts .* B_k{i};
            GBar{i} = eye(3);
            for n = 1 : i
                GBar{i} = G_k{n} * GBar{i};
            end
            for j = 1 : Nc
                if (j < i)
                    HHat{i,j} = zeros(3);
                    for m = j : i-1
                        Mat = eye(3);
                        for n = m : i-1
                            Mat = G_k{n+1} * Mat;
                        end
                        HHat{i,j} = HHat{i,j} + Mat * H_k{m};
                    end
                    HHat{i,j} = HHat{i,j} + H_k{i};
                elseif (j == i)
                    HHat{i,j} = H_k{i};
                else
                    HHat{i,j} = zeros(3);
                end
            end
            HBar{i} = HHat{i,1};
        end
        %-------------【GPNN_QP_SOLVER】
        ZTilde(:,k) = G_k{1} * (Z_2W(:,k-1) - Zr_2W(:,k-1)) + H_k{1} * UTilde(:,k-1);
        HH = 2 .* ((cell2mat(HHat)).' * QTilde * cell2mat(HHat) + RTilde);
        gg = 2 .* (cell2mat(HHat)).' * QTilde * (cell2mat(GBar) * ZTilde(:,k) + cell2mat(HBar) * UTilde(:,k-1));
        SS = [-cell2mat(HHat); cell2mat(HHat); -EyeTilde; EyeTilde; -eye(3*Nc); eye(3*Nc)];
        hh = [-ZHat_min + (cell2mat(GBar) * ZTilde(:,k) + cell2mat(HBar) * UTilde(:,k-1)); ...
               ZHat_max - (cell2mat(GBar) * ZTilde(:,k) + cell2mat(HBar) * UTilde(:,k-1)); ...
              -UHat_min + OnesTilde * UTilde(:,k-1); UHat_max - OnesTilde * UTilde(:,k-1); ...
              -DeltaUHat_min; DeltaUHat_max];
        UDeltaPre = FUNCSET.GPNN4QP(HH,gg,SS,hh,[]);	% △U_pre
        %-------------【Output of MPC】
        DeltaUTilde(:,k) = UDeltaPre(1:Nu,end);         % △U
        UTilde(:,k) = UTilde(:,k-1) + DeltaUTilde(:,k);
        Um(:,k) = Ur(:,k) + UTilde(:,k);
        Zd_2B(:,k) = GammaBody_rhyth(k) .* Um(:,k);
        [RelFootPose,JointAngle] = FUNCSET.STRIDE(legL,Measure,Alpha,Theta_0,beta_0,phi_0, Um(:,k),Zd_2B(:,k), SwiSta,nsw, GammaFoot_horiz_rhyth(k),GammaFoot_verti_rhyth(k));
        Pd_2B{legL}(:,k) = RelFootPose;
        
        %--------------------------------------------------------------------------
        % SMC
        %--------------------------------------------------------------------------
        U_SMC{legL}(:,k) = Kp \ (De_deriv{legL}(:,k-1) + Ki * e{legL}(:,k-1)) + DPd{legL}(:,k-1) + sigma .* sign(s{legL}(:,k-1));
        invJacobian = FUNCSET.INVJACOBI(Measure,alphaL,q_const{legL}(:,k-1));           % Inverse Jacobian
        Dq_ideal{legL}(:,k) = invJacobian * ( U_SMC{legL}(:,k) + Disturbance(:,k) );    % Plant
        q_ideal{legL}(1:3,k) = q_ideal{legL}(1:3,k-1) + Dq_ideal{legL}(:,k) * ts;
        q_ideal{legL}(4,k) = - q_ideal{legL}(2,k) - q_ideal{legL}(3,k);
        q_const{legL}(:,k) = FUNCSET.ANGLESAT(q_ideal{legL}(:,k));
        Pa{legL}(:,k) = FUNCSET.FORKEN(Measure,alphaL,q_const{legL}(:,k));
        Pd{legL}(:,k) = Pd_2B{legL}(:,k);
        DPd{legL}(:,k) = (Pd{legL}(:,k) - Pd{legL}(:,k-1)) ./ ts;
        e{legL}(:,k) = Pd{legL}(:,k) - Pa{legL}(:,k);
        e_integ{legL}(:,k) = e_integ{legL}(:,k-1) + e{legL}(:,k) .* ts;
        e_deriv{legL}(:,k) = (e{legL}(:,k) - e{legL}(:,k-1)) ./ ts;
        De_deriv{legL}(:,k) = (e_deriv{legL}(:,k) - e_deriv{legL}(:,k-1)) ./ ts;
        s{legL}(:,k) = Kp * e{legL}(:,k) + Ki * e_integ{legL}(:,k) + e_deriv{legL}(:,k);
        Ds{legL}(:,k) = (s{legL}(:,k) - s{legL}(:,k-1)) ./ ts;
        q_a{legL} = [q_a{legL}, q_const{legL,1}(:,k)];
        P_2B{legL}(:,k) = Pa{legL}(:,k);
    end
    
    %--------------------------------------------------------------------------
    % Feedback corresponding to MPC
    %--------------------------------------------------------------------------
    % Solve the actual relative pose of COM according to the relative positions of stance phases (the number of stance phases must be greater than 1)
    Z_2B(:,k) = FUNCSET.SOLVBODYPOSE(Measure,Alpha,Theta_0, k,Um,nsw,SwiSta,P_2B);
    % Compute the actual absolute pose of COM w.r.t. the world frame
    k0 = max((nst-1) * Nstride, 1);	% Sampling time corresponding to the current reference body frame
    Z_2W(3,k) = Z_2W(3,k0) + Z_2B(3,k);
    Z_2W(1:2,k) = Z_2W(1:2,k0) + rot2(Z_2W(3,k0)) * Z_2B(1:2,k);
    % Compute the actual absolute position of FOOT w.r.t. the world frame
    for legL = 1 : 6
        P_2W{legL}(1:2,k) = Z_2W(1:2,k) + rot2(Z_2W(3,k)) * P_2B{legL}(1:2,k);
        P_2W{legL}(3,k) = P_2B{legL}(3,k);
    end
end


%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
Zr_2W_trial{trial} = Zr_2W;
Z_2W_trial{trial} = Z_2W;
ZTilde_trial{trial} = ZTilde;
Pd_trial{trial} = Pd;
Pa_trial{trial} = Pa;
P_2W_trial{trial} = P_2W;
q_a_trial{trial} = q_a;
end
% End random trials
%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
% Save data
save Comparison_Data\Z_2W_trial_MPC_SMC                Z_2W_trial
save Comparison_Data\ZTilde_trial_MPC_SMC              ZTilde_trial
save Comparison_Data\Pd_trial_MPC_SMC                  Pd_trial
save Comparison_Data\Pa_trial_MPC_SMC                  Pa_trial
save Comparison_Data\P_2W_trial_MPC_SMC                P_2W_trial
save Comparison_Data\q_a_trial_MPC_SMC                 q_a_trial


%% 【Plot】
%{
fontSize = 12;                  % Font size in plot
fig101 = figure(101);    clf;	% Plot the actual absolute positions of the body and feet
set(gcf, 'Color','w');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
hold on;    box on;
plot3(Z_2W(1,:),Z_2W(2,:),bodyHeight*ones(1,Nrhyth), 'b-','LineWidth',5);
for legL = 1 : 6
    plot3(P_2W{legL}(1,:),P_2W{legL}(2,:),P_2W{legL}(3,:)+bodyHeight, 'LineWidth',2);
end
xlabel('X_w');    ylabel('Y_w');    zlabel('Z_w');
view([-1,-1,1]);
legend('Body','Foot1','Foot2','Foot3','Foot4','Foot5','Foot6', 'Location','northeast');
hold off;


figure(102);    clf;            % Plot the reference and actual absolute positions of the body and feet
set(gcf, 'Color','w');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
hold on;    box on;
plot(Zr_2W(1,:),Zr_2W(2,:), 'r-','LineWidth',2);
plot(Z_2W(1,:),Z_2W(2,:), 'g--','LineWidth',2);
hold off;


fig200 = figure(200);	clf;	% Plot the actual joint angles of the 6 legs
set(gcf, 'Color','w', 'Position',[680,100,560,600], 'Units','pixels');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
tl = tiledlayout(6,1, 'TileSpacing','none');  % Create block diagram layout and minimize subgraph spacing (R2019b and later versions are available)
for legL = 1 : 6
nexttile;                       % Create coordinate area object (R2019b and later versions are available)
hold on;    box on;
plot(TimeSeq,q_a{legL}(1,:), 'k-','LineWidth',2);
plot(TimeSeq,q_a{legL}(2,:), 'r-','LineWidth',2);
plot(TimeSeq,q_a{legL}(3,:), 'g-','LineWidth',2);
plot(TimeSeq,q_a{legL}(4,:), 'b-','LineWidth',2);
if (legL == 1)
    lgd = legend('q_1','q_2','q_3','q_4', 'Orientation','horizontal', 'Location','northoutside');
    set(lgd, 'FontSize',fontSize, 'FontName','Times New Roman');
end
if (legL < 6)
    set(gca,'XTickLabel',[]);   % Delete the X-axis label
end
yyaxis right;    ylabel(['Leg',num2str(legL)]);
set(gca,'YTickLabel',[], 'YColor','k', 'FontSize',fontSize, 'FontName','Times New Roman');
hold off;
end
xlabel(tl, 'Time (s)', 'FontSize',fontSize, 'FontName','Times New Roman');
ylabel(tl, 'Joint Angles (rad)', 'FontSize',fontSize, 'FontName','Times New Roman');
%}

%% 
disp('***** Program End！*****');
