%% System parameters
% Vehicle wheels' parameters
mw = 0.064;         % [Kg]          Wheel mass
Iw = 2.78742e-05;   % [Kg*m^2]      Wheel moment of inertia
rw = 0.032;         % [m]           Wheel radius
bw = 0.01;          % [N*m*s/rad]   Friction between the wheel and the ground 
kw = 100000;        % [N/m]         Ground and wheel stiffness
VehicleContactForce.w_offset = 0.005;   % Offset for Simscape Multibody simulation
% Vehicle chasis' parameters (vehicle's body)
mb = 0.5;           % [Kg]          Chasis mass
Ib = 0.00110817+0.5*0.06^2;     % [Kg*m^2]      Chasis moment of inertia
l = 0.06;        % [m]           Longitude to the vehicle's center of mass
% Other useful constants
g = 9.80665;        % [m/s^2]       Gravity acceleration

%% Incremental encoder parameters
QuadratureEncoder.PPR = 12;             % pulses per revolution (motor axis)
QuadratureEncoder.ResolutionMult = 4;   % flank reading mode (resolution multiplier)
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

accel.Bias = zeros(1, 3);        % [Hz]

%% Actuator parameters
MotorParameters.gearRatio = 34;     % gearbox ratio
MotorParameters.maxTorque = 0.85*9.81/100; % [Nm] max torque
MotorParameters.tau = 5e-3;         % torque modulator time constant