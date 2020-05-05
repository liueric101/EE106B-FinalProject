function [feas, zOpt, uOpt, JOpt] = OptimalControl(N, z0, sampleTime, VehicleParams, IneqConstraints, refPath)
%   Detailed explanation goes here

% number of states
nz = length(z0);
% number of inputs [longitudinal accel; steering angle]
nu = 2;

% Initialize yalmip variables
z = sdpvar(nz, N+1);
u = sdpvar(nu, N);

% initial condition constraint
constraints = z(:,1) == z0;

for i = 1:N
    cost = cost + posGain*norm(z(:,i)-refPath(:,i))^2;
    constraints = [constraints, ...
                   IneqConstraints.zMin - v <= z(:,i) <= IneqConstraints.zMax + v, ... % state constraints
                   IneqConstraints.uMin <= u(:,i) <= IneqConstraints.uMax, ...         % input constraints
                   z(:,i+1) == VehicleModel(z(:,i), u(:,i), sampleTime, VehicleParams)]; % state dynamics
end

options = sdpsettings('verbose', 0, 'solver', 'ipopt');
diagnostics = optimize(constraints, cost, options);

if (diagnostics.problem == 0)
    feas = true;
    zOpt = double(z);
    uOpt = double(u);
    JOpt = double(cost);
else
    feas = false;
    zOpt = [];
    JOpt = [];
    uOpt = [];
    return
end


end

