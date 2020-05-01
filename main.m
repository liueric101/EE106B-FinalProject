% Vehicle Dimensions 
% vehicle wheelbase
VehicleParams.Lf = 2; % [m]
VehicleParams.Lr = 2; % [m]

% yaw inertia
VehicleParams.Izz = 93343; % [kg*m^2]
% vehicle mass
VehicleParams.mass = 17130; % [kg]

%% MPC Parameters
% sampling time
sampleTime = 0.1; % [sec]
% MPC Horizon
M = 30;
% CFTOC Horizon
N = 5;
% initial conditions
% [x-pos; y-pos; speed; vehicle heading]
z0 = [0; 0; 30; 0]; % [m; m; m/s; rad]

%%
tic
% do the MPC
[feas, zOpt, uOpt, JOpt, pursuitPoints] = mpc(M, N, z0, vehiclePath, sampleTime, VehicleParams);
toc