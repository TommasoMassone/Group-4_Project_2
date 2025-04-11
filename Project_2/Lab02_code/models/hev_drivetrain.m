function [shaftSpd, shaftTrq, vehPrf] = hev_drivetrain(vehSpd, vehAcc, veh)
%hev_drivetrain drivetrain sub-model for the series HEV powertrain model
%   Implements a backward quasistatic model from the driving cycle to the
%   transmission input, including the vehicle longitudinal dynamics, wheels,
%   final drive.

%% Vehicle longitudinal dynamics
% Tractive effort (N)
vehResForce =  veh.body.f0 + veh.body.f1 .* vehSpd + veh.body.f2 .* vehSpd.^2;
vehForce = (vehSpd~=0) .* (vehResForce + veh.body.mass.*vehAcc);

%% Wheels
% Wheel speed (rad/s)
wheelSpd  = vehSpd ./ veh.wh.radius;
% Wheel torque (Nm)
wheelTrq = vehForce .* veh.wh.radius;
% Apply braking regulation
wheelTrq(vehForce<0) = 0.6 .* wheelTrq(vehForce<0);
% Final Drive
% Final drive input speed (rad/s)
shaftSpd = veh.fd.spdRatio .* wheelSpd;
% Final drive input torque (Nm)
shaftTrq  = wheelTrq ./ veh.fd.spdRatio;

%% Pack additional outputs
vehPrf.vehForce = vehForce;
vehPrf.shaftSpd = shaftSpd;
vehPrf.reqTrq = shaftTrq;