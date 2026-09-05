% model_preload_function.m
%
% Modellparameter im MATLAB Workspace setzen
%
% letzte Aenderung: 22.5.2019
clear;

% Streckendaten
global X;
global Y;


global l_v;
global l_h;
global w_ltr;
global l_l0;
global old_index_i;

global m;
global c_v;
global c_h;
global theta;
global v_y;

% Fahrzeugmasse
% Symbol: m, Einheit: kg
m = 1883.239;

% c_w - Wert des Fahrzeugs
% Symbol: c_w, Einheit: [dimensionslos]
c_w = 0.0;

% Luftdichte
% Symbol: rho_luft, Einheit: kg/(m^3)
rho_luft = 1.204;

% "Angriffsflache" fuer die Luft
% Symbol: A, Einheit: m^2
A = 1.9;

% Traegheitsmoment und die z-Achse
% Symbol: theta, Einheit: kg m
theta = 2529.4827;

% Abstand Gesamtschwerpunkt - Vorderachse
% Symbol: l_v, Einheit: m
l_v = 1.55;

% Abstand Gesamtschwerpunkt - Hinterachse
% Symbol:  l_h, Einheit: m
l_h = 1.05;

% Schraeglaufsteifigkeit - Vorderrad
% Symbol:  c_v, Einheit: N / rad
c_v = 169265.0;

% Schraeglaufsteifigkeit - Hinterrad
% Symbol:  c_h, Einheit: N / rad
c_h = 249962.5;


% Startgeschwindigkeit
% Symbol: v_0, Einheit: m/s
v_0 = 10;

% lateral velocity
v_y = 0;

% Sollgeschwindigkeit
% Symbol: v_soll, Einheit: m/s
v_soll = 10;

% Polverteilung Geschwindigkeitsregelung
pv_1  = -0.1;
pv_2  = -4.8; 

% Reglerparameter Geschwindigkeitsregelung
kpv = -pv_1-pv_2;
kiv = pv_1*pv_2;


% Startposition des Fahrzeugs (x)
% Symbol:  x_0
% Einheit: m
x_0 = 0;


% Startposition des Fahrzeugs (y)
% Symbol:  y_0
% Einheit: m
y_0 = 1.5;


% Startwinkel des Fahrzeugs
% Symbol:  psi_0
% Einheit: rad
psi_0 = 0;


% Abstand zum Punkt "P"
% Symbol:  l_l0
% Einheit: m
l_l0 = 10;


% Seitlicher Wunschabstand
% Symbol:  w_ltr
% Einheit: m
w_ltr = 0;

% Polverteilung Querdynamikregelung
pq_1  = -3;
pq_2  = -5;
pq_3  = -15; 

% Reglerparameter Querdynamikregelung
alpha_q_ = -1 * pq_1 * pq_2 * pq_3; 
alpha_q0 = pq_1*pq_2 + pq_1*pq_3 + pq_2*pq_3;
alpha_q1 = -1 * (pq_1 + pq_2 + pq_3);

% Positionsindex fuer X[i] bzw. Y[i]
old_index_i = 1; 

% global parameters for Sideslip Angle Estimation
global P_kd;
global P_dd;
global state_estimate_kd;
global state_estimate_dd;
global last_beta;
global analytical_solution_flag;
global yaw_rate_threshold;
global vel_x_threshold;

analytical_solution_flag = false;
P_kd = [0.1 0; 0 0.1];
P_dd = [0.1 0; 0 0.1];
state_estimate_kd = [v_0;0];
state_estimate_dd = [0;0];
last_beta = 0;
yaw_rate_threshold = 0.0001;
vel_x_threshold = 0.5;

% Variablen fuer Trajektorie initialisieren
trajectory_init;
% Trajektorie generieren
trajectory_generate;

X = X_mitte_rechts;
Y = Y_mitte_rechts;


