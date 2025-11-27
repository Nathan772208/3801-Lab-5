function wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state)
%
% Inputs:
%   wind_inertial = 3x1 wind vector expressed in the inertial reference frame
%                   [Wx; Wy; Wz] (m/s)
%   aircraft_state = 3x1 vector containing Euler angles:
%                    aircraft_state(1) = phi   (roll angle, rad)
%                    aircraft_state(2) = theta (pitch angle, rad)
%                    aircraft_state(3) = psi   (yaw angle, rad)
%
% Outputs:
%   wind_body = 3x1 wind vector expressed in the aircraft body frame (m/s)
%
% Methodology:
%   - Uses a sequence of 3-2-1 Euler rotations (roll, pitch, yaw) to construct
%     the direction cosine matrix (DCM) that transforms vectors from the
%     inertial frame to the aircraft body frame.
%   - Rotation matrices:
%       M1: rotation about body x-axis (roll)
%       M2: rotation about body y-axis (pitch)
%       M3: rotation about body z-axis (yaw)
%   - The full transformation from inertial to body coordinates is:
%         Inertial_to_Body = M1 * M2 * M3
%   - The body-frame wind vector is computed as:
%         wind_body = Inertial_to_Body * wind_inertial
%

phi = aircraft_state(1);
theta = aircraft_state(2);
psi = aircraft_state(3);

% Matrix 1: rotation about x-axis (roll)
M1 = [ 1      0           0;
       0  cos(phi)   sin(phi);
       0 -sin(phi)   cos(phi)];

% Matrix 2: rotation about y-axis (pitch)
M2 = [ cos(theta)   0   -sin(theta);
             0      1        0;
        sin(theta)  0    cos(theta)];

% Matrix 3: rotation about z-axis (yaw)
M3 = [ cos(psi)  sin(psi)   0;
      -sin(psi)  cos(psi)   0;
            0          0    1];

% Final DCM: product of sequential rotations
Inertial_to_Body = M1 * M2 * M3;

wind_body = Inertial_to_Body * wind_inertial;

end
