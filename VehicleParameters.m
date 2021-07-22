%% Parámetros del sistema
% Parámetros de las ruedas del vehículo
mw = 0.064;         % [Kg]          Masa de la rueda
Iw = 2.78742e-05;   % [Kg*m^2]      Momento de inercia de la rueda
rw = 0.032;         % [m]           Radio de la rueda
bw = 0.01;          % [N*m*s/rad]   Fricción de la rueda con el suelo
kw = 100000;        % [N/m]         Rigidez del suelo y rueda
w_offset = 0.005;
% Parámetros del chasis del vehículo
mb = 0.5;           % [Kg]          Masa del cuerpo del robot
Ib = 0.00110817+0.5*0.06^2;     % [Kg*m^2]      Momento de inercia del cuerpo del robot
l = 0.06;        % [m]           Longitud al CM del robot
% Otras constantes
g = 9.80665;        % [m/s^2]       Aceleración de la gravedad

%% Parámetros encoder incremental
encoderPPR = 12;    % Cantidad de pulsos por revolución del eje motor
encoderConf = 4;    % Modo de lectura de los flancos
delta = 2*pi/(encoderPPR*encoderConf);

%% Parámetros actuadores
i = 34;     % Relación de reducción
tau = 5e-3; % Constante de tiempo modulador de torque