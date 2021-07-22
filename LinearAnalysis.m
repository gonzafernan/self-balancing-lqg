%% Parámetros del sistema: Importar de data dictionary correspondiente
VehicleParamObj = ...
Simulink.data.dictionary.open('VehicleParametersDictionary.sldd');
dDataSectObj = getSection(VehicleParamObj, 'Design Data');
childNamesList = dDataSectObj.evalin('who');
for n = 1:numel(childNamesList)
  hEntry = dDataSectObj.getEntry(childNamesList{n});
  assignin('base', hEntry.Name, hEntry.getValue);
end
clear dDataSectObj childNamesList n hEntry VehicleParamObj
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
sys = ss(A, B, C, D);
clear E F G hT
%% Polos y ceros del modelo LTI, análisis de estabilidad a lazo abierto
% disp(eig(A));
disp(pole(sys));
pzplot(sys);
grid on
%% Sistema de fase no mínima: Cero real positivo
HTw_x = tf(sys(1, 1)); HTw_x
HTw_x.Name = "H(s)";
HTw_x.InputName = "T_w";
HTw_x.OutputName = "x_t";
disp(pole(HTw_x))
figure(1)
pzmap(HTw_x)
grid on