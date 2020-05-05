function [feas, zOpt, uOpt, JOpt] = mpc(M, N, z0, sampleTime, VehicleParams)
% MPC Summary of this function goes here
%   Detailed explanation goes here

% number of states
nz = length(z0);
% number of inputs [steering angle; tire force]
nu = 2;

% upper state constraints
% [X, Y, theta, psi, v_x, v_y, r]
constraints.zMax = [100; 4; pi; 15; 15; 1.5];
constraints.zMin = [0; -4; -pi; -15; -15; -1.5];
% lower input constraints
IneqConstraints.uMin = [-30*pi/180; -0.5];
% upper input constraints
IneqConstraints.uMax = [30*pi/180; 0.5];

% Variables to track
zOpt = zeros(nz, M+1);
uOpt = zeros(nu, M);
JOpt = zeros(1,M);
feas = false([1, M]);

refPath = zeros(nz, N+1);
refPath(4, :) = 15*ones(N+1);

% initial conditions
zOpt(:,1) = z0;
uOpt(:,1) = [0;0];


for i = 1:M
    fprintf('Working on MPC Iteration #%d \n', i);
    % Reference path is just center lane, zeros on every other 
    refPath(1, :) = zOpt(1,i):15:(zOpt(1,i)+15*(N-1));
    IneqConstraints.zMin = [zOpt(1,i); -6; 0; -2*pi];
    
    % do the optimal control problem to figure out next step
    [feas(i), z, u, cost] = OptimalControl(N, zOpt(:,i), sampleTime, VehicleParams, constraints, refPath);

    if ~feas(i)
        disp('Infeasible region reached!');
        return
    end
    % closed loop predictions
    uOpt(:,i) = [u(1,1); u(2,1)];
    JOpt(:,i) = cost;
    zOpt(:,i+1) = z(:,2);
end

end

