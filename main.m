clear;
clc;
close all;
%% init

fig = 1;



% params
wind_inertial = [0;0;0;];
ttwistor % defines aircraft_parameters

%% Problem 2

tspan = [0 100];

aircraft_state_0 = zeros(12,3);
aircraft_surfaces = zeros(4,3);

% in:[Xe, Ye, Ze, phi, theta, psi, Ue, Ve, We, p, q, r]
aircraft_state_0(:,1) = [0,0,-1609.34, 0,0,0, 15,0,0, 0,0,0]';
aircraft_surfaces(:,1) = [0,0,0,0]';
aircraft_state_0(:,2) = [0,0,-1800, 0,0.0278,0, 20.99,0,0.5837, 0,0,0]';
aircraft_surfaces(:,2) = [0.1079,0,0,0.3182]';
aircraft_state_0(:,3) = [0,0,-1800, 15*pi/180,-12*pi/180,270*pi/180, 19,3,-2, 0.08*pi/180,-0.2*pi/180,0]';
aircraft_surfaces(:,3) = [5*pi/180,2*pi/180,-13*pi/180,0.3*pi/180]';

for i=1:3
    [time, aircraft_state] = ode45(@(time,aircraft_state) AircraftEOM(time, ...
        aircraft_state, aircraft_surfaces(:,i), wind_inertial, aircraft_parameters), ...
        tspan, aircraft_state_0(:,i));

    PlotAircraftSim(time,aircraft_state',aircraft_surfaces(:,i),fig:fig+5,'b');
    fig = fig+6;
end

%% Problem 3.1
if (problem_3_1)



end
%% Problem 3.2
if (problem_3_2)



end
