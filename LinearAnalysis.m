%% Cargar parámetros del sistema en workspace
VehicleParameters;

%% Modelo simplificado lineal invariante en el tiempo LTI
% Ecuación matricial dinámica lineal: E*qdd + F*qd + G*q = hT*u
E = [
    2*mw + 2*Iw/rw^2 + mb,  mb*l;
    mb*l,                   Ib + mb*l^2
];
F = [
    2*bw/rw^2,  0;
    0,          0
];
G = [
    0,  0;
    0,  -mb*g*l
];
hT = [
   1/rw,    0;
   -1,      1
];

% Representación de modelo LTI en espacio de estados
% Matriz dinámica del sistema
A = [
    zeros(2, 2),    eye(2);
    -inv(E)*G,      -inv(E)*F
];
% Matriz de entradas
B = [
    zeros(2, 2);
    E\hT
];
% Matriz de salidas medidas 
% (caso hipotético de realimentación completa de estado)
C = eye(4);
% Matriz de transferencia directa
D = [
    0, 0;
    0, 0; 
    0, 0; 
    0, 0
];
% Construcción de modelo LTI en tiempo continuo
ltiSys = ss(A, B, C, D);
clear E F G hT A B C D

%% Polos y ceros del modelo LTI, análisis de estabilidad a lazo abierto
% disp(eig(A));
disp(pole(ltiSys));
pzplot(ltiSys);
grid on

%% Sistema de fase no mínima: Cero real positivo
HTw_x = tf(ltiSys(1, 1));
HTw_x.Name = "H(s)";
HTw_x.InputName = "T_w";
HTw_x.OutputName = "x_t";
disp(pole(HTw_x))
figure(1)
pzmap(HTw_x)
grid on
clear HTw_x