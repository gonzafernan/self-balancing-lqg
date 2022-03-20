%% Loading system parameters on workspace
VehicleParameters;

%% Linear time-invariant simplified model (LTI) 
% Linear dynamic matrix equation: E*qdd + F*qd + G*q = hT*u
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

% State-space representation of the LTI model
% State or system matrix
A = [
    zeros(2, 2),    eye(2);
    -inv(E)*G,      -inv(E)*F
];
% Input matrix
B = [
    zeros(2, 2);
    E\hT
];
% Output matrix
% (hypothetical case with full-state feedback)
C = eye(4);
% Feedthrough (or feedforward) matrix
D = [
    0, 0;
    0, 0; 
    0, 0; 
    0, 0
];
% LTI system construction in continuous time
ltiSys = ss(A, B, C, D);
clear E F G hT A B C D

%% Poles and zeros of the LTI system, open-loop stability analysis
% disp(eig(A));
disp(pole(ltiSys));
pzplot(ltiSys);
grid on

%% Non-minimum phase system: Real positive zero
HTw_x = tf(ltiSys(1, 1));
HTw_x.Name = "H(s)";
HTw_x.InputName = "T_w";
HTw_x.OutputName = "x_t";
disp(pole(HTw_x))
figure(1)
pzmap(HTw_x)
grid on
clear HTw_x