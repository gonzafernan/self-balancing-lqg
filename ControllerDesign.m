%% Parámetros del sistema: Importar de data dictionary correspondiente
VehicleParamObj = ...
Simulink.data.dictionary.open('VehicleModelDictionary.sldd');
dDataSectObj = getSection(VehicleParamObj, 'Design Data');
ltiSysObj = dDataSectObj.getEntry('ltiSys');
ltiSys = getValue(ltiSysObj);  % Modelo LTI
% Importar controlador LQG
lqgControllerObj = dDataSectObj.getEntry('lqgController');
lqgController = getValue(lqgControllerObj);
%% Análisis de controlabilidad del sistema
Wc = ctrb(ltiSys);
% Wc = [B, A*B, A^2*B, A^3*B]; % Idem
disp(rank(Wc));
clear Wc

%% Controlador por realimentación completa de estado
% Asignación de polos manual
p_des = [0; -40.3815; -80; -7.5366];
lqgController.K = place(ltiSys.A, ltiSys.B(:, 1), p_des);
disp(lqgController.K)
disp(eig(ltiSys.A-ltiSys.B(:, 1)*lqgController.K))
setValue(lqgControllerObj, lqgController);
saveChanges(VehicleParamObj);
clear p_des 

%% Regulador cuadrático lineal LQR
alpha1  = sqrt(0.25);       % alpha asociado a posición traslación
x1max   = 0.01;           % traslación máxima (?)
q1 = alpha1^2/x1max^2;

alpha2  = sqrt(0.25);       % alpha asociado a ángulo inclinación
x2max   = deg2rad(20);      % ángulo de inclinación máximo admisible
q2 = alpha2^2/x2max^2;

alpha3  = sqrt(0.25);       % alpha asociado a velocidad de traslación
wm_max  = 5*2*pi/60;      % velocidad angular máxima del motor
x3max   = 0.90*wm_max*0.032;   % velocidad de traslación máxima del robot (90%)
%x3max = 0.02;
q3 = alpha3^2/x3max^2;

alpha4  = sqrt(0.25);       % alpha asociado a velocidad angular de inclinación
x4max   = 1*2*pi/60;  % velocidad angular de inclinación máxima
q4 = alpha4^2/x4max^2;
Qx = eye(4); 
Qx(1, 1) = q1; Qx(2, 2) = q2; Qx(3, 3) = q3; Qx(4, 4) = q4;

Tmax = 0.85/100;            % Torque máximo de carga de los actuadores
u1max = 2*0.90*Tmax;        % Valor máximo de entrada admisible (90% torque máximo)
beta1 = 1;
rho1 = beta1^2/u1max^2;     % Regla de Bryson para acción de control
%rho1 = 1;
Qu = rho1;                  % Matriz de costo de acción de control   

lqgController.K = lqr(ltiSys.A, ltiSys.B(:, 1), Qx, Qu);
disp(lqgController.K)
% Ubicación de los polos con control LQR
disp(eig(ltiSys.A-ltiSys.B(:, 1)*lqgController.K))
setValue(lqgControllerObj, lqgController);
saveChanges(VehicleParamObj);

%% Análisis de observabilidad del sistema
rw = getValue(dDataSectObj.getEntry('rw'));
QuadratureEncoder = getValue(dDataSectObj.getEntry('QuadratureEncoder'));
MotorParameters = getValue(dDataSectObj.getEntry('MotorParameters'));
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
setValue(lqgControllerObj, lqgController);
saveChanges(VehicleParamObj);
% Matriz de observabilidad
Wo = obsv(ltiSys);
% Wo = [C; C*A; C*A^2; C*A^3]; % Idem
disp(rank(Wo));
clear Wo