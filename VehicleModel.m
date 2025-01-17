function [zOut] = VehicleModel(z,u,dt,VehicleParams)
% VEHICLEMODEL 
% Inputs:
% z - 6x1 vector - states of the vehicle. 
% [X, Y, psi, v_x, v_y, r]
% [global X pos, global Y pos, global yaw angle, longitudinal velocity,
% lateral velocity, yaw rate]
% 
% u - 2x1 vector - inputs to vehicle.
% [delta, Fx] 
% [Steering angle, longitudinal rear force]
zOut = sdpvar(6,1);

Lf = VehicleParams.Lf;
Lr = VehicleParams.Lr;
C = VehicleParams.C;
B = VehicleParams.B;
delta = u(1);
Fx = u(2);

%TODO Fiala tire models.
alphaF = atan((z(5)+Lf*z(6))/z(4)) - delta;
alphaR = atan((z(5)-Lr*z(6))/z(4));
Fyf = 1;
Fyr = 1;


%Global X pos X
zOut(1) = z(1) + dt*(v_x*sin(psi) +v_y*cos(psi));
% Global Y Pos Y
zOut(2) = z(2) + dt*(v_x*cos(psi) +v_y*sin(psi));
% Global Yaw Angle psi
zOut(3) = z(3) + dt*z(6);
% Body longitudinal velocity v_x
zOut(4) = z(4) + dt*((1/m)*(Fx - Fyf*sin(delta))+z(5)*z(6));
% Body lateral velocity v_y
zOut(5) = z(5) + dt*((1/m)*(Fyf*cos(delta)+Fyr)-z(4)*z(6));
% Yaw rate r
zOut(6) = z(6) + dt*(1/VehicleParams.Iz)*(Lf*Fyf*cos(delta)- ...
    Lr*Fyr);

end

