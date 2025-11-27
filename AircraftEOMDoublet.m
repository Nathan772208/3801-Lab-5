function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, aircraft_parameters)
%
% Inputs:
%   time                = current simulation time (s)
%   aircraft_state      = 12x1 state vector:
%                         [xe; ye; ze; phi; theta; psi; ue; ve; we; p; q; r]
%                         positions in inertial frame (m), Euler angles (rad),
%                         body-frame linear velocities (m/s), and body
%                         angular rates (rad/s)
%   aircraft_surfaces   = control surface deflections vector
%                         aircraft_surfaces(1) = elevator deflection (rad)
%                         remaining entries may include aileron, rudder, throttle
%   doublet_size        = magnitude of control surface doublet input (rad)
%   doublet_time        = duration of each half of the doublet input (s)
%   wind_inertial       = 3x1 wind vector in inertial frame [Wx; Wy; Wz] (m/s)
%   aircraft_parameters = structure containing aircraft constants:
%                         m, g, Ix, Iy, Iz, Ixz
%
% Outputs:
%   xdot = 12x1 derivative of the state vector:
%          [xe_dot; ye_dot; ze_dot; phi_dot; theta_dot; psi_dot;
%           ue_dot; ve_dot; we_dot; p_dot; q_dot; r_dot]
%
% Methodology:
%   - Implements the full six degree-of-freedom (6-DOF) rigid-body equations
%     of motion for a fixed-wing aircraft.
%   - A control surface doublet is applied to the elevator channel:
%         time <= doublet_time: elevator = elevator + doublet_size
%         doublet_time < time <= 2*doublet_time: elevator = elevator - doublet_size
%         time > 2*doublet_time: elevator returns to baseline
%   - Atmospheric density is computed from altitude using atmoscoesa.
%   - Aerodynamic forces and moments (X, Y, Z, L, M, N) are obtained from
%     AeroForcesAndMoments, which accounts for aircraft state, control
%     inputs, wind, and atmospheric density.
%   - Computes inertia coupling using gamma parameters (gamma1 through gamma8)
%     derived from the inertia matrix.
%   - Computes translational accelerations based on aerodynamic forces,
%     rotation rates, and gravity.
%   - Computes rotational accelerations from aerodynamic moments and
%     inertia coupling.
%   - Computes inertial position and Euler angle rates using the standard
%     direction cosine relationships for a 3-2-1 rotation sequence.
%

% unpack state vector
xe = aircraft_state(1);
ye = aircraft_state(2);
ze = aircraft_state(3);
phi = aircraft_state(4);
theta = aircraft_state(5);
psi = aircraft_state(6);
ue = aircraft_state(7);
ve = aircraft_state(8);
we = aircraft_state(9);
p = aircraft_state(10);
q = aircraft_state(11);
r = aircraft_state(12);

% Define all constants
m = aircraft_parameters.m;
g = aircraft_parameters.g;
Ix = aircraft_parameters.Ix;
Iy = aircraft_parameters.Iy;
Iz = aircraft_parameters.Iz;
Ixz = aircraft_parameters.Ixz;

% Define all gamma (Inertia) terms
gamma = Ix*Iz - Ixz^2;
gamma1 = (Ixz/gamma) * (Ix - Iy + Iz);
gamma2 = (Iz*(Iz - Iy) + Ixz^2) / gamma;
gamma3 = Iz / gamma;
gamma4 = Ixz / gamma;
gamma5 = (Iz - Ix) / Iy;
gamma6 = Ixz / Iy;
gamma7 = (Ix*(Ix - Iy) + Ixz^2) / gamma;
gamma8 = Ix / gamma;

% Calculate density from altitude (negative ze = positive altitude)
[~,~,~,density] = atmoscoesa(-ze);

% Elevator doublet input modification
if (time <= doublet_time)
    aircraft_surfaces(1) = aircraft_surfaces(1) + doublet_size;
end

if (time > doublet_time && time <= (2*doublet_time))
    aircraft_surfaces(1) = aircraft_surfaces(1) - doublet_size; 
end

if (time > (2*doublet_time))
    aircraft_surfaces(1) = aircraft_surfaces(1);
end

% Aerodynamic forces and moments
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

% Euler angle derivatives
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
