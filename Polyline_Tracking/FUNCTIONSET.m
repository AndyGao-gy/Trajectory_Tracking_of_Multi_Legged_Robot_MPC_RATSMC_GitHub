%**************************************************************************
%*******************¡¾Author£ºAndyGao£»Date£º2020/10¡¿*********************
%**************************************************************************
%% Define functions
function FUNCSET = FUNCTIONSET()
    FUNCSET.ROBOTMEASURES = @ ROBOTMEASURES;
    FUNCSET.FORKEN = @ FORKEN;
    FUNCSET.INVKEN = @ INVKEN;
    FUNCSET.STRIDE = @ STRIDE;
    FUNCSET.GPNN4QP = @ GPNN4QP;
    FUNCSET.dynamicGPNN = @ dynamicGPNN;
    FUNCSET.SOLVBODYPOSE = @ SOLVBODYPOSE;
    FUNCSET.INVJACOBI = @ INVJACOBI;
    FUNCSET.ANGLESAT = @ ANGLESAT;
end


%% ¡¾Robot's structural parameters¡¿
function [Measure,Alpha] = ROBOTMEASURES()
% Measure = [Link1, Link2, Link3, Link4, BodyRadius] (m)
Measure = [0.09, 0.15, 0.16, 0.15, 0.18];
% Azimuth angles (rad)
Alpha = [pi/6, pi/2, 5*pi/6, -5*pi/6, -pi/2, -pi/6];
end


%% ¡¾Forward Kenimatics¡¿
function Position = FORKEN(Measure,alphaL,q)
L1 = Measure(1);    L2 = Measure(2);    L3 = Measure(3);    L4 = Measure(4);    BR = Measure(5);
q1 = q(1);    q2 = q(2);    q23 = q(2)+q(3);    q234 = q(2)+q(3)+q(4);
S_L1 = sin(alphaL + q1);    C_L1 = cos(alphaL + q1);
%----------------------------------------------------------------------
Orientation = [ C_L1 * sin(q234),   C_L1 * cos(q234),   S_L1
                S_L1 * sin(q234),   S_L1 * cos(q234),   -C_L1
                -cos(q234),         sin(q234),          0	];
Position = [BR*cos(alphaL) + (L1+L2*cos(q2) + L3*sin(q23) + L4*sin(q234)) * C_L1;
            BR*sin(alphaL) + (L1+L2*cos(q2) + L3*sin(q23) + L4*sin(q234)) * S_L1;
            L2*sin(q2) - L3*cos(q23) - L4*cos(q234)                             ];
end


%% ¡¾InverseKenimatics function¡¿ (in the general configuration)
function Theta_ik = INVKEN(Measure,alphaL,phi_i,beta_i,FootPosition)
L1 = Measure(1);    L2 = Measure(2);    L3 = Measure(3);    L4 = Measure(4);    BR = Measure(5);
footPositionX = FootPosition(1);
footPositionY = FootPosition(2);
footPositionZ = FootPosition(3);

theta1 = atan2(footPositionY - BR*sin(alphaL), footPositionX - BR*cos(alphaL)) - alphaL; % Joint 1 is unlocked
%--------------------------------------------------------------------------
tempA = footPositionZ + L4*cos(beta_i-sign(alphaL)*phi_i);
tempB = (footPositionY - BR*sin(alphaL))/sin(alphaL+theta1) - L1 - L4*sin(beta_i-sign(alphaL)*phi_i);
tempC = L2;
tempD = -L3;
%--------------------------------------------------------------------------
theta3 = asin((tempA^2+tempB^2-tempC^2-tempD^2) / (-2*tempC*tempD));
theta2 = atan((-2*tempC*tempD*cos(theta3)) / (tempA^2+tempB^2+tempC^2-tempD^2)) + atan2(tempA , tempB);
theta4 = beta_i - sign(alphaL)*phi_i - theta2 - theta3;
%--------------------------------------------------------------------------
Theta_ik = [theta1; theta2; theta3; theta4];
end


%% ¡¾Gait mode generator¡¿
function [StriRelFootPose,StriTheta] = STRIDE(L,Measure,Alpha,Theta_0,beta_0,phi_0, Stride_t, Stride_i, SwiSur,sc, GammaFoot_swig,GammaFoot_lift)
%**************************************************************************
% Set foot motion parameters during single stride period
%**************************************************************************
global liftH Xstride Ystride Zrotate
liftH = 0.10; % Lift height of swing phase (Unit: m)
% Desired total stride of COM in three free directions
Xstride = Stride_t(1);
Ystride = Stride_t(2);
Zrotate = Stride_t(3);
% Instantaneous stride of COM in three free directions
Xstride_i = Stride_i(1);
Ystride_i = Stride_i(2);
Zrotate_i = Stride_i(3);

%**************************************************************************
% Foot position w.r.t. the instantaneous body frame during single stride period
%**************************************************************************
alphaL = Alpha(L); % Limb's azimuth angle
% Initial foot position w.r.t. the body frame
FootPosition_0 = FORKEN(Measure,alphaL,Theta_0(L,:));
Px_0 = FootPosition_0(1);     Py_0 = FootPosition_0(2);    Pz_0 = FootPosition_0(3);
% Flag of swing phase and stance phase (1 for swing; -1 for stance)
sgnL = SwiSur(sc,L);
% Determine whether the current foot has swung and update foot position w.r.t. the initial body frame
Px_0c = Px_0;
Py_0c = Py_0;
if ( (sc > 1) && ismember(1, SwiSur(1:sc-1,L)) ) % True means that this foot has already swung
    Px_0c = Px_0 * cos(Zrotate) - Py_0 * sin(Zrotate) + Xstride;
    Py_0c = Px_0 * sin(Zrotate) + Py_0 * cos(Zrotate) + Ystride;
end
% Instantaneous foot coordinates w.r.t. the instantaneous body frame
StriRelFootPose(1,1) = (Px_0c - Xstride_i) * cos(Zrotate_i) + (Py_0c - Ystride_i) * sin(Zrotate_i) ...
    + (1+sgnL)/2 * GammaFoot_swig * (Px_0 * cos(Zrotate - Zrotate_i) - Py_0 * sin(Zrotate - Zrotate_i) - (Px_0 - Xstride) * cos(Zrotate_i) - (Py_0 - Ystride) * sin(Zrotate_i));
StriRelFootPose(2,1) =-(Px_0c - Xstride_i) * sin(Zrotate_i) + (Py_0c - Ystride_i) * cos(Zrotate_i) ...
    + (1+sgnL)/2 * GammaFoot_swig * (Px_0 * sin(Zrotate - Zrotate_i) + Py_0 * cos(Zrotate - Zrotate_i) + (Px_0 - Xstride) * sin(Zrotate_i) - (Py_0 - Ystride) * cos(Zrotate_i));
StriRelFootPose(3,1) = Pz_0 + (1+sgnL)/2 * liftH * GammaFoot_lift;

% Inverse Kinematics
Px_cur = StriRelFootPose(1,1);	% x-coordinate of the foot at the current time
Py_cur = StriRelFootPose(2,1);	% y-coordinate of the foot at the current time
Pz_cur = StriRelFootPose(3,1);	% z-coordinate of the foot at the current time
phi_i  = phi_0;                 % Instantaneous pitch angle of the body platform
beta_i = beta_0;                % Instantaneous relative rotation angle of the sucker
StriTheta = INVKEN(Measure,alphaL,phi_i,beta_i,[Px_cur;Py_cur;Pz_cur]); % Joint angles [q1;q2;q3;q4]

end


%% ¡¾Solve the QP based on GPNN¡¿ (Rely on ode45)
%**************************************************************************
% minimize    1/2 * X' * Q * X + c' * X
% subject to  d <= E * X <= h
%**************************************************************************
function X = GPNN4QP(Q,c,E,h,d)
[nm, ~] = size(E);
if (isempty(h)),    h = Inf .* ones(nm,1);  end
if (isempty(d)),    d = -Inf .* ones(nm,1); end

lammda = 50;
W = E / Q * E.';
R = Q \ E.';
a = - Q \ c;
q = E * a;

[t,U] = ode45(@(t,U) dynamicGPNN(t,U,W,q,lammda,h,d), [0,1], zeros(nm,1)); % Solution of dynamic equation of GPNN
X = R * U.' + a; % Optimal solution of QP
end


%% ¡¾Dynamic equation of GPNN¡¿
function dudt = dynamicGPNN(t,U,W,q,lammda,h,d)
temp = W * U + q - U;
Px = zeros(length(h),1);            % Projection function
for i = 1 : length(h)
    if (temp(i) < d(i))
        Px(i) = d(i);
    elseif (temp(i) > h(i))
        Px(i) = h(i);
    else
        Px(i) = temp(i);
    end
end
dudt = lammda .* (Px - W * U - q);	% Dynamic equation
end


%% ¡¾Velocity Jacobian of limb¡¿ (Consider only 3 translation DoFs)
function invJacobian = INVJACOBI(Measure,alphaL,q)
L1 = Measure(1);    L2 = Measure(2);    L3 = Measure(3);    L4 = Measure(4);    BR = Measure(5);
q1 = q(1);    q2 = q(2);    q23 = q(2)+q(3);    % q234 = q(2)+q(3)+q(4);
q234 = 0;    S_L1 = sin(alphaL + q1);    C_L1 = cos(alphaL + q1);
%----------------------------------------------------------------------
% Jac = [ -(L1+L2*cos(q2)+L3*sin(q23)+L4*sin(q234))*S_L1,     (-L2*sin(q2)+L3*cos(q23)+L4*cos(q234))*C_L1,    (L3*cos(q23)+L4*cos(q234))*C_L1,    (L4*cos(q234))*C_L1
%         (L1+L2*cos(q2)+L3*sin(q23)+L4*sin(q234))*C_L1,      (-L2*sin(q2)+L3*cos(q23)+L4*cos(q234))*S_L1,    (L3*cos(q23)+L4*cos(q234))*S_L1,    (L4*cos(q234))*S_L1
%         0,                                                  L2*cos(q2)+L3*sin(q23)+L4*sin(q234),            L3*sin(q23)+L4*sin(q234),           L4*sin(q234)
%         1,                                                  0,                                              0,                                  0                   ];
% invJacobian =  inv(Jac);
Jac = [ -(L1+L2*cos(q2)+L3*sin(q23)+L4*sin(q234))*S_L1,     (-L2*sin(q2)+L3*cos(q23)+L4*cos(q234))*C_L1,    (L3*cos(q23)+L4*cos(q234))*C_L1
         (L1+L2*cos(q2)+L3*sin(q23)+L4*sin(q234))*C_L1,     (-L2*sin(q2)+L3*cos(q23)+L4*cos(q234))*S_L1,    (L3*cos(q23)+L4*cos(q234))*S_L1
         0,                                                 L2*cos(q2)+L3*sin(q23)+L4*sin(q234),            L3*sin(q23)+L4*sin(q234)    ];
invJacobian = inv(Jac);
end


%% ¡¾Saturation constraints on joint angles¡¿
function q_const = ANGLESAT(q_desir)
%--------------------------------------------------------------------------
% Rotation range of each joint (Unit: rad)
theta1_min = (pi/180) * (-60);      theta1_max = (pi/180) * (60);
theta2_min = (pi/180) * (-90);      theta2_max = (pi/180) * (90);
theta3_min = (pi/180) * (-60);      theta3_max = (pi/180) * (80);
theta4_min = (pi/180) * (-90);      theta4_max = (pi/180) * (90);
%--------------------------------------------------------------------------
[row, col] = size(q_desir);
%--------------------------------------------------------------------------
% Saturation constraints
q_const = q_desir;
for c = 1 : col
    q_const(1,c) = max(theta1_min, min(theta1_max, q_desir(1,c)));
    q_const(2,c) = max(theta2_min, min(theta2_max, q_desir(2,c)));
    q_const(3,c) = max(theta3_min, min(theta3_max, q_desir(3,c)));
    q_const(4,c) = max(theta4_min, min(theta4_max, q_desir(4,c)));
end
end


%% ¡¾Solve the relative pose of COM according to the relative positions of stance phases¡¿ (the number of stance phases must be greater than 1)
function RelBodyPose = SOLVBODYPOSE(Measure,Alpha,Theta_0, timek,Stride,sk,SwiSur,RelFootPose)
Sur_i = find(SwiSur(sk,:) < 0);	% Look for the stance phase in the current swing period
P_0 = zeros(2); % Initial relative positions of two stance phases (The first column corresponds to the x and y coordinates of the first foot)
P_i = zeros(2); % Instantaneous relative positions of two stance phases (The first column corresponds to the x and y coordinates of the first foot)
for i = 1 : 2
    L = Sur_i(i);               % Limb number
    alphaL = Alpha(L);          % % Limb's azimuth angle
    FootPosition_0 = FORKEN(Measure,alphaL,Theta_0(L,:));	% Initial position of foot w.r.t. the body frame
    P_0(:,i) = FootPosition_0(1:2,1);
    if ( (sk > 1) && ismember(1, SwiSur(1:sk-1,L)) )        % True means that this foot has already swung
        P_0(1,i) = FootPosition_0(1,1) * cos(Stride(3,timek)) - FootPosition_0(2,1) * sin(Stride(3,timek)) + Stride(1,timek);
        P_0(2,i) = FootPosition_0(1,1) * sin(Stride(3,timek)) + FootPosition_0(2,1) * cos(Stride(3,timek)) + Stride(2,timek);
    end
    P_i(:,i) = RelFootPose{L}(1:2,timek);
end
solution3 = atan2((P_0(2,1)-P_0(2,2))*(P_i(1,1)-P_i(1,2)) - (P_0(1,1)-P_0(1,2))*(P_i(2,1)-P_i(2,2)), (P_0(1,1)-P_0(1,2))*(P_i(1,1)-P_i(1,2)) + (P_0(2,1)-P_0(2,2))*(P_i(2,1)-P_i(2,2)));
solution1 = P_0(1,2) - P_i(1,2)*cos(solution3) + P_i(2,2)*sin(solution3);
solution2 = P_0(2,2) - P_i(1,2)*sin(solution3) - P_i(2,2)*cos(solution3);
RelBodyPose = [solution1; solution2; solution3];
end

