%% Cargar parámetros del sistema y análisis lineal en workspace
VehicleParameters;
LinearAnalysis;

%% Análisis de controlabilidad del sistema
Wc = ctrb(ltiSys);
% Wc = [B, A*B, A^2*B, A^3*B]; % Idem
disp(rank(Wc));
clear Wc

%% Controlador por realimentación completa de estado
% Asignación de polos manual
p_des = [-5; -40.3815; -80; -7.5366];
lqgController.K = place(ltiSys.A, ltiSys.B(:, 1), p_des);
disp(lqgController.K)
disp(eig(ltiSys.A-ltiSys.B(:, 1)*lqgController.K))
clear p_des 

%% Regulador cuadrático lineal LQR
alpha1  = sqrt(0.25);       % alpha asociado a posición traslación
x1max   = 0.01;           % traslación máxima (?)
q1 = alpha1^2/x1max^2;
clear alpha1 x1max

alpha2  = sqrt(0.25);       % alpha asociado a ángulo inclinación
x2max   = deg2rad(20);      % ángulo de inclinación máximo admisible
q2 = alpha2^2/x2max^2;
clear alpha2 x2max

alpha3  = sqrt(0.25);           % alpha asociado a velocidad de traslación
wm_max  = 5*2*pi/60;            % velocidad angular máxima del motor
x3max   = 0.90*wm_max*0.032;    % velocidad de traslación máxima del robot (90%)
q3 = alpha3^2/x3max^2;
clear alpha3 wm_max x3max

alpha4  = sqrt(0.25);       % alpha asociado a velocidad angular de inclinación
x4max   = 1*2*pi/60;        % velocidad angular de inclinación máxima
q4 = alpha4^2/x4max^2;
clear alpha4 x4max

% Matriz de costo de estado
Qx = eye(4);
Qx(1, 1) = q1; Qx(2, 2) = q2; Qx(3, 3) = q3; Qx(4, 4) = q4;

Tmax = 0.85/100;            % Torque máximo de carga de los actuadores
u1max = 2*0.90*Tmax;        % Valor máximo de entrada admisible (90% torque máximo)
beta1 = 1;
rho1 = beta1^2/u1max^2;     % Regla de Bryson para acción de control
clear Tmax u1max beta1 q1 q2 q3 q4

% Matriz de esfuerzo de acción de control
Qu = rho1;                  % Matriz de costo de acción de control   
clear rho1

% Obtención de controlador LQR
lqgController.K = lqr(ltiSys.A, ltiSys.B(:, 1), Qx, Qu);
disp(lqgController.K)
% Ubicación de los polos con control LQR
disp(eig(ltiSys.A-ltiSys.B(:, 1)*lqgController.K))
clear Qx Qu

%% Análisis de sistema de fase no mínima
lqrSys = feedback(ltiSys(:, 1), lqgController.K);
pzmap(lqrSys(1, 1))
grid on

%% Análisis de observabilidad del sistema
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
% Actualizar sistema con nuevas matrices C y D
ltiSys = ss(ltiSys.A, ltiSys.B, C, D);
% Matriz de observabilidad
Wo = obsv(ltiSys);
% Wo = [C; C*A; C*A^2; C*A^3]; % Idem
disp(rank(Wo));
clear Wo C D

%% Observador por asignación de autovalores
% Asignación de polos
p_des = [-1; -2; -4; -8];
lqgController.L = place(ltiSys.A', ltiSys.C', p_des)';
disp(lqgController.L)
disp(eig(ltiSys.A-lqgController.L*ltiSys.C))
clear p_des

%% Varianza de sensores
delta = 2*pi/(QuadratureEncoder.PPR*QuadratureEncoder.ResolutionMult);
sigma2_phi_m = delta^2/12;  % Varianza encoder incremental
sigma2_theta = (accel.RNoiseDensity)^2*10; % Varianza acelerómetro
sigma2_theta_d = (gyro.RNoiseDensity*pi/180)^2*10; % Varianza giróscopo
clear delta

%% Estimador lineal cuadrático LQE
% Matriz de covarianza para ruido de proceso
Rw = [
    sigma2_phi_m,       0,              0;
    0,                  sigma2_theta,   0;
    0,                  0,              sigma2_theta_d
];
clear sigma2_phi_m sigma2_theta sigma2_theta_d
% Matriz de covarianza para ruido de proceso
Rv = 0.1;
% Diseño del observador óptimo
[L, P, E] = lqe(ltiSys.A, ltiSys.B(:, 2), ltiSys.C, Rv, Rw);
%disp(L)
lqgController.L = L;
disp(E)
clear Rw Rv L P E