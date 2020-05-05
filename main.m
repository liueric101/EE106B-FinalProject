% All lengths in meters, velocities in meter/second, mass in kg.

% Vehicle Dimensions 
% vehicle wheelbase
VehicleParams.Lf = 2; 
VehicleParams.Lr = 2; 

%Coefficient of friction
VehicleParams.mu = 0.5;

% Fiala Coefficients
VehicleParams.C = ;
VehicleParams.B = ;
% yaw inertia
VehicleParams.Iz = 93343; % [kg*m^2]
% vehicle mass
VehicleParams.mass = 17130; % [kg]

%% MPC Parameters
% sampling time
sampleTime = 0.1; % [sec]
% MPC Horizon
M = 10;
% CFTOC Horizon
N = 5;
% initial conditions
% [X, Y, psi, v_x, v_y, r]
% [global X pos, global Y pos, global yaw angle, longitudinal velocity,
% lateral velocity, yaw rate]
z0 = [0; 0; pi/4; 3; 10; 0]; % [m; m; rad; m/s; m/s; rad/s]

%%
tic
% do the MPC
[feas, zOpt, uOpt, JOpt, pursuitPoints] = mpc(M, N, z0, vehiclePath, sampleTime, VehicleParams);
toc