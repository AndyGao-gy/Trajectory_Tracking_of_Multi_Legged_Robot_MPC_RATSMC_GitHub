%% Curve-shaped Reference Trajectory
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


%% 【Parameters setting of single swing period and single stride period】
% Flag of swing phase and stance phase (1 for swing; -1 for stance)
gait_flag = 3;          % Gait mode of the robot (3 or 4 or 5)
switch gait_flag
    case {3},   SwiSta = [1 -1 1 -1 1 -1;  -1 1 -1 1 -1 1];
    case {4},   SwiSta = [1 -1 -1 1 -1 -1;  -1 1 -1 -1 1 -1;  -1 -1 1 -1 -1 1];
    case {5},   SwiSta = [1 -1 -1 -1 -1 -1;  -1 1 -1 -1 -1 -1;  -1 -1 1 -1 -1 -1;  -1 -1 -1 1 -1 -1;  -1 -1 -1 -1 1 -1;  -1 -1 -1 -1 -1 1];
    otherwise,  error('!!! Gait mode Error !!!');
end
numSwingPer = size(SwiSta,1);       % Number of swing period during single stride period
floorfactor = gait_flag / 6;        % Floor factor
dutyfactor = 1 - floorfactor;       % Duty factor

%**************************************************************************
% Single swing period
%**************************************************************************
Tswing = 1;                         % Duration of a single swing period
Nswing = 100;                       % Interpolation points of a single swing period【Must be consistent with vrep_main.m】
ts = Tswing / Nswing;               % Sample interval (Time step size)
TimeSeq_Swing = linspace(ts,Tswing, Nswing)'; % Sampling time sequences of a single swing period
% Sigmoid and Sinusoid interpolation functions in single swing period
const = 20;                         % Parameter of Sigmoid
Tg = Tswing / 2;                    % The given time parameter of Sigmoid
GammaFoot_horiz = 1 ./ (1 + exp(-const .* (TimeSeq_Swing - Tg)));	% Sigmoid function
GammaFoot_verti = -0.5 * cos(2*pi/Tswing * TimeSeq_Swing) + 0.5;	% Sinusoid function

%**************************************************************************
% Single stride period
%**************************************************************************
Nstride = numSwingPer*Nswing;       % Interpolation points of a single stride period
GammaFoot_horiz_strid = [];         % Foot interpolation in horizontal direction during single stride period
GammaFoot_verti_strid = [];         % Foot interpolation in vertical direction during single stride period
GammaBody_strid = [];               % Body interpolation during single stride period
for r = 1 : numSwingPer
    GammaFoot_horiz_strid = [GammaFoot_horiz_strid; GammaFoot_horiz];
    GammaFoot_verti_strid = [GammaFoot_verti_strid; GammaFoot_verti];
    GammaBody_strid = [GammaBody_strid; (r-1)*dutyfactor + GammaFoot_horiz*dutyfactor];
end
DGammaBody_strid = [0; (GammaBody_strid(2:end) - GammaBody_strid(1:end-1)) ./ ts]; % Derivative of the body interpolation during single stride period


%% 【Directed curvilinear reference trajectory】
%**************************************************************************
% PART 1
%**************************************************************************
numStridePer_1 = 18;                                    % Number of stride period during rhythmic movement
Tend_rhyth_1 = numSwingPer * Tswing * numStridePer_1;   % End time of rhythmic movement
Nrhyth_1 = Nstride * numStridePer_1;                    % Interpolation points of rhythmic movement
TimeSeq_rhyth_1 = linspace(ts,Tend_rhyth_1, Nrhyth_1);	% Sampling time sequences of rhythmic movement

GammaFoot_horiz_rhyth_1 = kron(ones(1,numStridePer_1), GammaFoot_horiz_strid'); % Foot interpolation in horizontal direction during rhythmic movement
GammaFoot_verti_rhyth_1 = kron(ones(1,numStridePer_1), GammaFoot_verti_strid'); % Foot interpolation in vertical direction during rhythmic movement
GammaBody_rhyth_1 = kron(ones(1,numStridePer_1), GammaBody_strid');             % Body interpolation during rhythmic movement
DGammaBody_rhyth_1 = kron(ones(1,numStridePer_1), DGammaBody_strid');           % Derivative of the body interpolation during rhythmic movement

Ur_1 = [0.08; 0.00; deg2rad(10)] * ones(1,Nrhyth_1);    % Reference control input [Sx; Sy; Sz]
Zr_2B_1 = zeros(3,Nrhyth_1);                            % Reference relative pose of COM w.r.t. the initial body frame [xB_r; yB_r; thB_r]
Zr_2W_1 = zeros(3,Nrhyth_1);                            % Reference absolute pose of COM w.r.t. the world frame [X_r; Y_r; TH_r]
Pr_2B_1 = cell(6,1);                                    % Reference relative position of FOOT w.r.t. the instantaneous body frame
Pr_2W_1 = cell(6,1);                                    % Reference absolute position of FOOT w.r.t. the world frame
q_r_1 = cell(6,1);                                      % Reference joint angles [q1;q2;q3;q4]

Zr_2W_1(:,1) = [0; 0; 0];                               % Reference initial absolute pose of COM
for kr_1 = 1 : Nrhyth_1
    nst = floor((kr_1-1) / Nstride) + 1;                % Number of stride period corresponding to the current moment
    nsw = floor(((kr_1-1) - Nstride * (nst-1)) / Nswing) + 1; % Number of swing period corresponding to the current moment
    %--------------------------------------------------------------------------
    % Relative Pose
    %--------------------------------------------------------------------------
    Zr_2B_1(:,kr_1) = Ur_1(:,kr_1) .* GammaBody_rhyth_1(kr_1);
    for legL = 1 : 6
        [RelFootPose,JointAngle] = FUNCSET.STRIDE(legL,Measure,Alpha,Theta_0,beta_0,phi_0, Ur_1(:,kr_1), Zr_2B_1(:,kr_1), SwiSta,nsw, GammaFoot_horiz_rhyth_1(kr_1),GammaFoot_verti_rhyth_1(kr_1));
        Pr_2B_1{legL}(:,kr_1) = RelFootPose;
        q_r_1{legL}(:,kr_1) = JointAngle;
    end
    %--------------------------------------------------------------------------
    % Absolute Pose
    %--------------------------------------------------------------------------
    Zr_2W_1(3,kr_1) = Zr_2W_1(3,max(1,(nst-1)*Nstride)) + Zr_2B_1(3,kr_1);
    Zr_2W_1(1:2,kr_1) = Zr_2W_1(1:2,max(1,(nst-1)*Nstride)) + rot2(Zr_2W_1(3,max(1,(nst-1)*Nstride))) * Zr_2B_1(1:2,kr_1);
    for legL = 1 : 6
        Pr_2W_1{legL}(1:2,kr_1) = Zr_2W_1(1:2,kr_1) + rot2(Zr_2W_1(3,kr_1)) * Pr_2B_1{legL}(1:2,kr_1);
        Pr_2W_1{legL}(3,kr_1) = Pr_2B_1{legL}(3,kr_1);
    end
end

%**************************************************************************
% PART 2
%**************************************************************************
numStridePer_2 = 18;                                    % Number of stride period during rhythmic movement
Tend_rhyth_2 = numSwingPer * Tswing * numStridePer_2;   % End time of rhythmic movement
Nrhyth_2 = Nstride * numStridePer_2;                    % Interpolation points of rhythmic movement
TimeSeq_rhyth_2 = linspace(ts,Tend_rhyth_2, Nrhyth_2);	% Sampling time sequences of rhythmic movement

GammaFoot_horiz_rhyth_2 = kron(ones(1,numStridePer_2), GammaFoot_horiz_strid'); % Foot interpolation in horizontal direction during rhythmic movement
GammaFoot_verti_rhyth_2 = kron(ones(1,numStridePer_2), GammaFoot_verti_strid'); % Foot interpolation in vertical direction during rhythmic movement
GammaBody_rhyth_2 = kron(ones(1,numStridePer_2), GammaBody_strid');             % Body interpolation during rhythmic movement
DGammaBody_rhyth_2 = kron(ones(1,numStridePer_2), DGammaBody_strid');           % Derivative of the body interpolation during rhythmic movement

Ur_2 = [0.08; 0.00; deg2rad(-10)] * ones(1,Nrhyth_2);    % Reference control input [Sx; Sy; Sz]
Zr_2B_2 = zeros(3,Nrhyth_2);                            % Reference relative pose of COM w.r.t. the initial body frame [xB_r; yB_r; thB_r]
Zr_2W_2 = zeros(3,Nrhyth_2);                            % Reference absolute pose of COM w.r.t. the world frame [X_r; Y_r; TH_r]
Pr_2B_2 = cell(6,1);                                    % Reference relative position of FOOT w.r.t. the instantaneous body frame
Pr_2W_2 = cell(6,1);                                    % Reference absolute position of FOOT w.r.t. the world frame
q_r_2 = cell(6,1);                                      % Reference joint angles [q1;q2;q3;q4]

Zr_2W_2(:,1) = Zr_2W_1(:,end);
for kr_2 = 1 : Nrhyth_2
    nst = floor((kr_2-1) / Nstride) + 1;                % Number of stride period corresponding to the current moment
    nsw = floor(((kr_2-1) - Nstride * (nst-1)) / Nswing) + 1; % Number of swing period corresponding to the current moment
    %--------------------------------------------------------------------------
    % Relative Pose
    %--------------------------------------------------------------------------
    Zr_2B_2(:,kr_2) = Ur_2(:,kr_2) .* GammaBody_rhyth_2(kr_2);
    for legL = 1 : 6
        [RelFootPose,JointAngle] = FUNCSET.STRIDE(legL,Measure,Alpha,Theta_0,beta_0,phi_0, Ur_2(:,kr_2), Zr_2B_2(:,kr_2), SwiSta, nsw, GammaFoot_horiz_rhyth_2(kr_2), GammaFoot_verti_rhyth_2(kr_2));
        Pr_2B_2{legL}(:,kr_2) = RelFootPose;
        q_r_2{legL}(:,kr_2) = JointAngle;
    end
    %--------------------------------------------------------------------------
    % Absolute Pose
    %--------------------------------------------------------------------------
    Zr_2W_2(3,kr_2) = Zr_2W_2(3,max(1,(nst-1)*Nstride)) + Zr_2B_2(3,kr_2);
    Zr_2W_2(1:2,kr_2) = Zr_2W_2(1:2,max(1,(nst-1)*Nstride)) + rot2(Zr_2W_2(3,max(1,(nst-1)*Nstride))) * Zr_2B_2(1:2,kr_2);
    for legL = 1 : 6
        Pr_2W_2{legL}(1:2,kr_2) = Zr_2W_2(1:2,kr_2) + rot2(Zr_2W_2(3,kr_2)) * Pr_2B_2{legL}(1:2,kr_2);
        Pr_2W_2{legL}(3,kr_2) = Pr_2B_2{legL}(3,kr_2);
    end
end


%% 【Complete reference trajectory】
numStridePer = numStridePer_1 + numStridePer_2;     % Number of stride period during rhythmic movement
Tend_rhyth = Tend_rhyth_1 + Tend_rhyth_2;           % End time of rhythmic movement
Nrhyth = Nrhyth_1 + Nrhyth_2;                       % Interpolation points of rhythmic movement
TimeSeq_rhyth = linspace(ts,Tend_rhyth, Nrhyth);    % Sampling time sequences of rhythmic movement
GammaFoot_horiz_rhyth = [GammaFoot_horiz_rhyth_1, GammaFoot_horiz_rhyth_2];	% Foot interpolation in horizontal direction during rhythmic movement
GammaFoot_verti_rhyth = [GammaFoot_verti_rhyth_1, GammaFoot_verti_rhyth_2]; % Foot interpolation in vertical direction during rhythmic movement
GammaBody_rhyth = [GammaBody_rhyth_1, GammaBody_rhyth_2];                   % Body interpolation during rhythmic movement
DGammaBody_rhyth = [DGammaBody_rhyth_1, DGammaBody_rhyth_2];                % Derivative of the body interpolation during rhythmic movement
Ur = [Ur_1, Ur_2];                                  % Reference control input [Sx; Sy; Sz]
Zr_2B = [Zr_2B_1, Zr_2B_2];                         % Reference relative pose of COM w.r.t. the initial body frame [xB_r; yB_r; thB_r]
Zr_2W = [Zr_2W_1, Zr_2W_2];                         % Reference absolute pose of COM w.r.t. the world frame [X_r; Y_r; TH_r]
Pr_2B = cell(6,1);                                  % Reference relative position of FOOT w.r.t. the instantaneous body frame
Pr_2W = cell(6,1);                                  % Reference absolute position of FOOT w.r.t. the world frame
q_r = cell(6,1);                                    % Reference joint angles [q1;q2;q3;q4]
for legL = 1 : 6
    Pr_2B{legL} = [Pr_2B_1{legL}, Pr_2B_2{legL}];
    Pr_2W{legL} = [Pr_2W_1{legL}, Pr_2W_2{legL}];
    q_r{legL} = [q_r_1{legL}, q_r_2{legL}];
end


%% 【Plot】
fontSize = 12;          % Font size in plot
fig100 = figure(100);    clf;
set(gcf, 'Color','w');
set(gca, 'FontSize',fontSize, 'FontName','Times New Roman');
hold on;    box on;
plot3(Zr_2W(1,:),Zr_2W(2,:),bodyHeight*ones(1,Nrhyth), 'b-','LineWidth',5);
for legL = 1 : 6
    plot3(Pr_2W{legL}(1,:),Pr_2W{legL}(2,:),Pr_2W{legL}(3,:), 'LineWidth',2);
end
xlabel('X_w');    ylabel('Y_w');    zlabel('Z_w');
view([-1,-1,1]);
legend('Body','Foot1','Foot2','Foot3','Foot4','Foot5','Foot6', 'Location','northeast');
hold off;


%% 【Save and Export】
save ReferenceTrajectory\SwiSta                     SwiSta                  % Flag of swing phase and stance phase (1 for swing; -1 for stance)
save ReferenceTrajectory\ts                         ts                      % Sample interval (Time step size)
save ReferenceTrajectory\Nswing                     Nswing                  % Interpolation points of a single swing period【Must be consistent with vrep_main.m】
save ReferenceTrajectory\Nstride                    Nstride                 % Interpolation points of a single stride period
save ReferenceTrajectory\Nrhyth                     Nrhyth                  % Interpolation points of rhythmic movement
save ReferenceTrajectory\TimeSeq_rhyth              TimeSeq_rhyth           % Sampling time sequences of rhythmic movement
save ReferenceTrajectory\GammaFoot_horiz_rhyth      GammaFoot_horiz_rhyth   % Foot interpolation in horizontal direction during rhythmic movement
save ReferenceTrajectory\GammaFoot_verti_rhyth      GammaFoot_verti_rhyth   % Foot interpolation in vertical direction during rhythmic movement
save ReferenceTrajectory\GammaBody_rhyth            GammaBody_rhyth         % Body interpolation during rhythmic movement
save ReferenceTrajectory\DGammaBody_rhyth           DGammaBody_rhyth        % Derivative of the body interpolation during rhythmic movement
save ReferenceTrajectory\Ur                         Ur                      % Reference control input [Sx; Sy; Sz]
save ReferenceTrajectory\Zr_2W                      Zr_2W                   % Reference absolute pose of COM w.r.t. the world frame [X_r; Y_r; TH_r]
save ReferenceTrajectory\Pr_2B                      Pr_2B                   % Reference relative position of FOOT w.r.t. the instantaneous body frame

%--------------------------------------------------------------------------
% Export the reference absolute body position
%--------------------------------------------------------------------------
ReferAbsBodyPosition = [Zr_2W(1:2,:)', bodyHeight * ones(1,Nrhyth)'];
% csvwrite('ReferAbsBodyPosition.csv', ReferAbsBodyPosition);


%% 
disp('***** Program End！*****');
