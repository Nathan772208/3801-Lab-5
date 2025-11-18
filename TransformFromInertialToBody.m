function wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state)
%TRANSFORMFROMINERTIALTOBODY Summary of this function goes here
%   Detailed explanation goes here

phi = aircraft_state(1);
theta = aircraft_state(2);
psi = aircraft_state(3);

R_EB = [cos(theta)*cos(psi),cos(theta)*sin(psi),-sin(theta);
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),sin(phi)*cos(theta);
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),cos(phi)*cos(theta)];

wind_body = R_EB*wind_inertial;

end

