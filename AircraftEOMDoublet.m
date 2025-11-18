function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, ...
    doublet_time, wind_inertial, aircraft_parameters)
%{
This function calculates the equations of motion for the fixed-wing aircraft with constant control
surfaces. The inputs are time, the 12 x 1 aircraft state vector, the 4 x 1 control surface vector, the 3 x 1 inertial
wind velocity in inertial coordinates, and the aircraft parameter structure. The output is the derivative of the
state vector

%}





zE = aircraft_state(3);

phi = aircraft_state(4);
theta = aircraft_state(5);
psi = aircraft_state(6);

uE = aircraft_state(7);
vE = aircraft_state(8);
wE = aircraft_state(9);

p = aircraft_state(10);
q = aircraft_state(11);
r = aircraft_state(12); 

Ixx = aircraft_parameters.Ix; %[kg m^2]
Iyy = aircraft_parameters.Iy; %[kg m^2]
Izz = aircraft_parameters.Iz; %[kg m^2]
Ixz = aircraft_parameters.Ixz; %[kg m^2]

m = aircraft_parameters.m; % kg
g = aircraft_parameters.g; % m/s^2
W = aircraft_parameters.W; % N

[~, ~, ~, density] = atmoscoesa(-zE);

de = aircraft_surfaces(1,1);
% da = aircraft_surfaces(2,1);
% dr = aircraft_surfaces(3,1);
% dt = aircraft_surfaces(4,1);

% apply doublet to elevator
if time <= doublet_time
    new_de = de + doublet_size;
elseif time <= 2 * doublet_time
    new_de = de - doublet_size;
else
    new_de = de;
end

% update elevator value before passing surfaces into AeroForcesAndMoments
aircraft_surfaces(1,1) = new_de;

[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

X = aero_forces(1);
Y = aero_forces(2);
Z = aero_forces(3);

L = aero_moments(1);
M = aero_moments(2);
N = aero_moments(3);


% Direction cosine matrix (for inertial to body transformation)
C_theta = cos(theta);
S_theta = sin(theta);

C_phi = cos(phi);
S_phi = sin(phi);

C_psi = cos(psi);
S_psi = sin(psi);

% Position kinematics (inertial rates)
pos_dot = [ ...
    C_theta*C_psi, S_phi*S_theta*C_psi - C_phi*S_psi, C_phi*S_theta*C_psi + S_phi*S_psi; ...
    C_theta*S_psi, S_phi*S_theta*S_psi + C_phi*C_psi, C_phi*S_theta*S_psi - S_phi*C_psi; ...
    -S_theta,      S_phi*C_theta,                   C_phi*C_theta ...
];
inertial_rates = pos_dot * [uE; vE; wE];

% Attitude kinematics
T = [ ...
    1,      sin(phi)*tan(theta),  cos(phi)*tan(theta); ...
    0,      cos(phi),             -sin(phi); ...
    0,      sin(phi)/cos(theta),  cos(phi)/cos(theta) ...
];
euler_dot = T * [p; q; r];

% Body accelerations
body_accel = [ ...
    r*vE - q*wE; ...
    p*wE - r*uE; ...
    q*uE - p*vE ...
] + g * [-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)] + (1/m)*[X; Y; Z];

% Inertia coefficients (from formulas)
Gamma = Ixx*Izz - Ixz^2;
Gamma_1 = Ixz*(Ixx - Iyy + Izz)/Gamma;
Gamma_2 = (Izz*(Izz - Iyy) + Ixz^2)/Gamma;
Gamma_3 = Izz/Gamma;
Gamma_4 = Ixz/Gamma;
Gamma_5 = (Izz - Ixx)/Iyy;
Gamma_6 = Ixz/Iyy;
Gamma_7 = (Ixx*(Ixx - Iyy) + Ixz^2)/Gamma;
Gamma_8 = Ixx/Gamma;

% Angular accelerations
ang_accel = [ ...
    Gamma_1*p*q - Gamma_2*q*r;
    Gamma_5*p*r - Gamma_6*(p^2 - r^2);
    Gamma_7*p*q - Gamma_1*q*r
] + [ ...
    Gamma_3*L + Gamma_4*N;
    (1/Iyy)*M;
    Gamma_4*L + Gamma_8*N
];

% Display equations for verification
% disp('Position rates:'); disp(inertial_rates);
% disp('Euler rates:'); disp(euler_dot);
% disp('Body acceleration:'); disp(body_accel);
% disp('Angular acceleration:'); disp(ang_accel);

% Coefficient display
% disp('Inertia coefficients:');
% disp(Gamma_1);
% disp(Gamma_2);
% disp(Gamma_3);
% disp(Gamma_4);
% disp(Gamma_5);
% disp(Gamma_6);
% disp(Gamma_7);
% disp(Gamma_8);
% disp(Gamma);

%% Controls


xdot = [inertial_rates; euler_dot; body_accel; ang_accel];

end