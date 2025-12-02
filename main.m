%{
Names: Ryan Chen, Luke Marsh, Barabara De Figueiredo Vera, Daniel
Gruenbauer
Group: 2-11
Lab: 5 - 3801
}%


clear;
clc;
close all;
% init

fig = 1;



% params
wind_inertial = [0;0;0;];
ttwistor % defines aircraft_parameters

%% Problem 2

tspan = [0 200];

aircraft_state_0 = zeros(12,3);
aircraft_surfaces = zeros(4,3);

% in:[Xe, Ye, Ze, phi, theta, psi, Ue, Ve, We, p, q, r]
aircraft_state_0(:,1) = [0,0,-1609.34, 0,0,0, 15,0,0, 0,0,0]';
aircraft_surfaces(:,1) = [0,0,0,0]';
aircraft_state_0(:,2) = [0,0,-1800, 0,0.0278,0, 20.99,0,0.5837, 0,0,0]';
aircraft_surfaces(:,2) = [0.1079,0,0,0.3182]';
aircraft_state_0(:,3) = [0,0,-1800, 15*pi/180,-12*pi/180,270*pi/180, 19,3,-2, 0.08*pi/180,-0.2*pi/180,0]';
aircraft_surfaces(:,3) = [5*pi/180,2*pi/180,-13*pi/180, 0.3]';

for i=1:3
    [time, aircraft_state] = ode45(@(time,aircraft_state) AircraftEOM(time, ...
        aircraft_state, aircraft_surfaces(:,i), wind_inertial, aircraft_parameters), ...
        tspan, aircraft_state_0(:,i));

    PlotAircraftSim(time,aircraft_state',repmat(aircraft_surfaces(:,i), 1, numel(time)),fig:fig+5,'b');
    fig = fig+6;
end

%% Problem 3.1
tspan = [0 3];
aircraft_state_0 = [0,0,-1800, 0,0.0278,0, 20.99,0,0.5837, 0,0,0]';
aircraft_surfaces = [0.1079,0,0,0.3182]';


u_0 = aircraft_state_0(7);
doublet_size = deg2rad(15);
doublet_time = 0.25; % s

de = aircraft_surfaces(1);


[time, aircraft_state] = ode45(@(time,aircraft_state) AircraftEOMDoublet(time, ...
    aircraft_state, aircraft_surfaces, doublet_size, ...
    doublet_time, wind_inertial, aircraft_parameters), ...
        tspan, aircraft_state_0);

    pos_inds = find(time <= 0.25);
    neg_inds = find(time >0.25 & time <=0.5);
    zero_inds = find(time > 0.5);

    surface_de(pos_inds) = de+doublet_size;

    surface_de(neg_inds) = de-doublet_size;
    surface_de(zero_inds) = de;

    surfaces = repmat(aircraft_surfaces, 1, numel(time));
    surfaces(1,:) = surface_de;


    PlotAircraftSim(time,aircraft_state', surfaces, fig:fig+5,'b');
    fig = fig+6;



%% calculating A matrix for short period approx
C_z_alpha = -aircraft_parameters.CD0 - aircraft_parameters.CLalpha;
rho = atmoscoesa(-mean(aircraft_state(3,:)));
Z_w = 0.5*rho*u_0*aircraft_parameters.S*C_z_alpha;
M_w = 0.5*rho*u_0*aircraft_parameters.S*aircraft_parameters.c*aircraft_parameters.Cmalpha;
M_wdot = 0.25*rho*aircraft_parameters.c^2 * aircraft_parameters.S * aircraft_parameters.Cmalphadot;
M_q = 0.25*rho*u_0*aircraft_parameters.c^2 * aircraft_parameters.S * aircraft_parameters.Cmq;

m = aircraft_parameters.m;
g = aircraft_parameters.g;
I_y = aircraft_parameters.Iy;


A_sp = [Z_w/m, u_0;   (1/I_y)*(M_w + (M_wdot*Z_w)/m),  (1/I_y)*(M_q + M_wdot*u_0)]


% Calculate the characteristic polynomial
char_poly = charpoly(A_sp)

w_n = sqrt(char_poly(3))

damping = char_poly(2)/(2*w_n)

%% Problem 3.2

tspan = [0 100];
aircraft_state_0 = [0,0,-1800, 0,0.0278,0, 20.99,0,0.5837, 0,0,0]';
aircraft_surfaces = [0.1079,0,0,0.3182]';
doublet_size = deg2rad(15);
doublet_time = 0.25; % s


[time, aircraft_state] = ode45(@(time,aircraft_state) AircraftEOMDoublet(time, ...
    aircraft_state, aircraft_surfaces, doublet_size, ...
    doublet_time, wind_inertial, aircraft_parameters), ...
        tspan, aircraft_state_0);

    new_pos_inds = find(time <= 0.25);
    new_neg_inds = find(time >0.25 & time <=0.5);
    new_zero_inds = find(time > 0.5);

    new_surface_de(new_pos_inds) = de+doublet_size;

    new_surface_de(new_neg_inds) = de-doublet_size;
    new_surface_de(new_zero_inds) = de;

    new_surfaces = repmat(aircraft_surfaces, 1, numel(time));
    new_surfaces(1,:) = new_surface_de;

    PlotAircraftSim(time,aircraft_state',new_surfaces,fig:fig+5,'m');
    fig = fig+6;


% TODO: define X_u and Z_u and then it'll run
X_u = ;


Z_u = ; 

A_phugoid = [X_u/m, -aircraft_parameters.g;     -Z_u/(m*u_0), 0]

% Calculate the characteristic polynomial
char_poly = charpoly(A_sp)

w_n = sqrt(char_poly(3))

damping = char_poly(2)/(2*w_n)



