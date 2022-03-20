%% Loading system parameters on workspace and linear analysis
VehicleParameters;
LinearAnalysis;

%% System controllability
Wc = ctrb(ltiSys);
% Wc = [B, A*B, A^2*B, A^3*B]; % Same
disp(rank(Wc));
clear Wc

%% Full state feedback controller (FSF)
% Pole placement by hand
p_des = [-5; -40.3815; -80; -7.5366];
lqgController.K = place(ltiSys.A, ltiSys.B(:, 1), p_des);
disp(lqgController.K)
disp(eig(ltiSys.A-ltiSys.B(:, 1)*lqgController.K))
clear p_des 

%% Linear-quadratic regulator (LQR)
alpha1  = sqrt(0.25);       % alpha associated with translation position
x1max   = 0.01;             % max translation required
q1 = alpha1^2/x1max^2;
clear alpha1 x1max

alpha2  = sqrt(0.25);       % alpha associated with tilt angle 
x2max   = deg2rad(20);      % max tilt angle required
q2 = alpha2^2/x2max^2;
clear alpha2 x2max

alpha3  = sqrt(0.25);          % alpha associated with translation velocity
wm_max  = 5*2*pi/60;           % max motor's angular velocity required
x3max   = 0.90*wm_max*0.032;   % max vehicle's translation velocity required (90%)
q3 = alpha3^2/x3max^2;
clear alpha3 wm_max x3max

alpha4  = sqrt(0.25);       % alpha associated with tilt angular velocity
x4max   = 1*2*pi/60;        % max tilt angular velocity required 
q4 = alpha4^2/x4max^2;
clear alpha4 x4max

% State weighting matrix
Qx = eye(4);
Qx(1, 1) = q1; Qx(2, 2) = q2; Qx(3, 3) = q3; Qx(4, 4) = q4;

Tmax = 0.85/100;            % max torque required on actuators
u1max = 2*0.90*Tmax;        % max input value admissible (90% max torque)
beta1 = 1;
rho1 = beta1^2/u1max^2;     % Bryson's rule for control action
clear Tmax u1max beta1 q1 q2 q3 q4

% Control weighting matrix
Qu = rho1; 
clear rho1

% Obtaining LQR controller
lqgController.K = lqr(ltiSys.A, ltiSys.B(:, 1), Qx, Qu);
disp(lqgController.K)
% Pole placement using LQR control
disp(eig(ltiSys.A-ltiSys.B(:, 1)*lqgController.K))
clear Qx Qu

%% Non-minimum phase system analysis
lqrSys = feedback(ltiSys(:, 1), lqgController.K);
pzmap(lqrSys(1, 1))
grid on

%% System observability
% C = [1, 0, 0, 0];
% C = [0, 1, 0, 0];
% C = [0, 0, 1, 0];
% C = [0, 0, 0, 1];
C = [
    QuadratureEncoder.PPR*QuadratureEncoder.ResolutionMult*...
    MotorParameters.gearRatio*(1/rw), 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 0, 1
    ];
% D = 0;
D = [
    0, 0; 
    0, 0; 
    0, 0
    ];
% Update system with new matices C and D
ltiSys = ss(ltiSys.A, ltiSys.B, C, D);
% Observability matrix
Wo = obsv(ltiSys);
% Wo = [C; C*A; C*A^2; C*A^3]; % Same
disp(rank(Wo));
clear Wo C D

%% Observer design by eigenvalue assignment
% Pole placement
p_des = [-1; -2; -4; -8];
lqgController.L = place(ltiSys.A', ltiSys.C', p_des)';
disp(lqgController.L)
disp(eig(ltiSys.A-lqgController.L*ltiSys.C))
clear p_des

%% Sensors variance
delta = 2*pi/(QuadratureEncoder.PPR*QuadratureEncoder.ResolutionMult);
sigma2_phi_m = delta^2/12;  % incremental encoder's variance
sigma2_theta = (accel.RNoiseDensity)^2*10;  % accelerometer's variance
sigma2_theta_d = (gyro.RNoiseDensity*pi/180)^2*10;  % gyroscope's variance
clear delta

%% Linear-quadratic estimator (LQE) aka Kalman Filter
% Covariance matrix for measurement noise
Rw = [
    sigma2_phi_m,       0,              0;
    0,                  sigma2_theta,   0;
    0,                  0,              sigma2_theta_d
];
clear sigma2_phi_m sigma2_theta sigma2_theta_d
% Covariance matrix for system noise
Rv = 0.1;
% Optimal observer design
[L, P, E] = lqe(ltiSys.A, ltiSys.B(:, 2), ltiSys.C, Rv, Rw);
%disp(L)
lqgController.L = L;
disp(E)
clear Rw Rv L P E