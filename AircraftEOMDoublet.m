function xdot = AircraftEOMDoublet(~, aircraft_state, aircraft_surfaces, doublet_size,...
                                   doublet_time, wind_inertial, aircraft_parameters)
% unpack state vector
xe = var(1);
ye = var(2);
ze = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
ue = var(7);
ve = var(8);
we = var(9);
p = var(10);
q = var(11);
r = var(12);

% Define all constants
m = aircraft_parameters.m;
g = aircraft_parameters.g;
Ix = aircraft_parameters.Ix;
Iy = aircraft_parameters.Iy;
Iz = aircraft_parameters.Iz;
Ixz = aircraft_parameters.Ixz;

% Define all gamma (Inertia) terms
gamma = Ix*Iz - Ixz^2;
gamma1 = (Ixz/gamma) * (Ix-Iy-Iz);
gamma2 = (Iz*(Iz-Iy) + Ixz^2)/gamma;
gamma3 = Iz/gamma;
gamma4 = Ixz/gamma;
gamma5 = (Iz-Ix)/Iy;
gamma6 = Ixz/Iy;
gamma7 = (Ix*(Ix-Iy) + Ixz^2)/gamma;
gamma8 = Ix/gamma;

% Calculate density from the height (inertial z)
density = stdatmo(z);

% Use the given AeroForcesAndMoments function to find X, Y, Z, L, M, N
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

% Unpack aero_forces vector
X = aero_forces(1);
Y = aero_forces(2);
Z = aero_forces(3);

% Unpack aero_moments vector
L = aero_moments(1);
M = aero_moments(2);
N = aero_moments(3);

% position derivatives
xe_dot = cos(theta)*cos(psi)*ue + (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*ve ...
    + (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*we;
ye_dot = cos(theta)*sin(psi)*ue + (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*ve ...
    + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*we;
ze_dot = -sin(theta)*ue + sin(phi)*cos(theta)*ve + cos(phi)*cos(theta)*we;

% euler angle derivatives
phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
theta_dot = cos(phi)*q - sin(phi)*r;
psi_dot = sin(phi)*sec(theta)*q + cos(phi)*sec(theta)*r;

% velocity derivatives
ue_dot = r*ve - q*we - g*sin(theta) + X/m;
ve_dot = p*we - r*ue + g*cos(theta)*sin(phi) + Y/m;
we_dot = q*ue - p*ve + g*cos(theta)*cos(phi) + Z/m;

% angular velocity derivatives
p_dot = (gamma1*p*q - gamma2*q*r) + (gamma3*L + gamma4*N);
q_dot = (gamma5*p*r - gamma6*(p^2 - r^2)) + (M/Iy);
r_dot = (gamma7*p*q - gamma1*q*r) + (gamma4*L + gamma8*N);

% combine for derivative state vector
xdot = [xe_dot;
    ye_dot;
    ze_dot;
    phi_dot;
    theta_dot;
    psi_dot;
    ue_dot;
    ve_dot;
    we_dot;
    p_dot;
    q_dot;
    r_dot];
end