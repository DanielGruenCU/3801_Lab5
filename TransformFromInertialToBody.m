function wind_body = TransformFromInertialToBody(wind_inertial, euler_angles)

% input angles
phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

% Define cos and sin shorthand
cphi   = cos(phi);
sphi   = sin(phi);
ctheta = cos(theta);
stheta = sin(theta);
cpsi   = cos(psi);
spsi   = sin(psi);

% Direction Cosine Matrix (DCM)
R_E_to_B = [ ctheta*cpsi,              sphi*stheta*cpsi - cphi*spsi,   cphi*stheta*cpsi + sphi*spsi;
                    ctheta*spsi,              sphi*stheta*spsi + cphi*cpsi,   cphi*stheta*spsi - sphi*cpsi;
                        -stheta,                   sphi*ctheta,                    cphi*ctheta ]';

wind_body = R_E_to_B*wind_inertial;

end