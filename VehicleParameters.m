%% Parámetros del sistema
% Parámetros de las ruedas del vehículo
mw = 0.064;         % [Kg]          Masa de la rueda
Iw = 2.78742e-05;   % [Kg*m^2]      Momento de inercia de la rueda
rw = 0.032;         % [m]           Radio de la rueda
bw = 0.01;          % [N*m*s/rad]   Fricción de la rueda con el suelo
kw = 100000;        % [N/m]         Rigidez del suelo y rueda
VehicleContactForce.w_offset = 0.005;   % Offset for Simscape Multibody simulation
% Parámetros del chasis del vehículo
mb = 0.5;           % [Kg]          Masa del cuerpo del robot
Ib = 0.00110817+0.5*0.06^2;     % [Kg*m^2]      Momento de inercia del chasis vehículo
l = 0.06;        % [m]           Longitud al CM del robot
% Otras constantes
g = 9.80665;        % [m/s^2]       Aceleración de la gravedad

%% Parámetros encoder incremental
QuadratureEncoder.PPR = 12;             % Cantidad de pulsos por revolución del eje motor
QuadratureEncoder.ResolutionMult = 4;   % Modo de lectura de los flancos
%delta = 2*pi/(QuadratureEncoder.PPR*QuadratureEncoder.ResolutionMult);

%% Gyroscope Specifications
% Gyroscope Sensitivity
gyro.S = 131;                % [LSB/(°/s)] FS_SEL=0  Full-Scale Range +-250 °/s
%gyro.S = 65.5;              % [LSB/(°/s)] FS_SEL=1  Full-Scale Range +-500 °/s
%gyro.S = 32.8;              % [LSB/(°/s)] FS_SEL=2  Full-Scale Range +-1000 °/s
%gyro.S = 16.4;              % [LSB/(°/s)] FS_SEL=3  Full-Scale Range +-2000 °/s

% Gyroscope Noise Performance
gyro.RNoiseDensity = 0.005;  % [°/s/sqrt(Hz)] Rate Noise Spectral Density

% Gyroscope Mechanical Frequencies
gyro.wnX = 30e3;            % [Hz]  X-Axis mechanical frequency (min)
gyro.wnY = 27e3;            % [Hz]  Y-Axis mechanical frequency (min)
gyro.wnZ = 24e3;            % [Hz]  Z-Axis mechanical frequency (min)
gyro.zeta = sqrt(2)/2;      % No value in datasheet

% Low Pass Filter Response
gyro.DLPF_w = 10*2*pi;       % [rad/s] 98 Hz cut-off frequency
gyro.DLPF_zeta = sqrt(2)/2;

% Output Data Rate
gyro.Fs = 1000;              % [Hz]

gyro.Bias = zeros(1, 3);
%% Accelerometer Specifications
% Accelerometer Sensitivity
accel.S = 16384;                 % [LSB/g] FS_SEL=0  Full-Scale Range +-2 g
%accel.S = 8192;                 % [LSB/g] FS_SEL=1  Full-Scale Range +-4 g
%accel.S = 4096;                 % [LSB/g] FS_SEL=2  Full-Scale Range +-8 g
%accel.S = 2048;                 % [LSB/g] FS_SEL=3  Full-Scale Range +-16 g

% Accelerometer Noise Performance
accel.RNoiseDensity = 400e-6;    % [g/sqrt(Hz)] Power Spectral Density

% Low Pass Filter Response
accel.DLPF_w = 10*2*pi;          % [rad/s] 94 Hz cut-off frequency
accel.DLPF_zeta = sqrt(2)/2;

% Output Data Rate
accel.Fs = 1000;                 % [Hz]

accel.Bias = zeros(1, 3);           % [Hz]

%% Parámetros actuadores
MotorParameters.gearRatio = 34;     % Relación de reducción
MotorParameters.tau = 5e-3; % Constante de tiempo modulador de torque