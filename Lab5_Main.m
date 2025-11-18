%% ASEN 3801 Lab 5
% Main Script
% Contributors:
% Chris Hattingh

%% Housekeeping
clear; clc; close all;
<<<<<<< Updated upstream
ttwistor;

%% Question Toggles
Q2_1 = 1;
Q2_2 = 1;
Q2_3 = 1;
Q3_1 = 0;
Q3_2 = 0;

%% Q2
=======
ttwistor

%% Question Toggles
Q2_1 = 0;
Q2_2 = 0;
Q2_3 = 0;
Q3_1 = 1;
Q3_2 = 1;

%% Q2
% Inital Values
x0_21 = [0;
         0;
         1609.34; % position (m)^
         0;
         0;
         0;       % orientation ^
         21;
         0;
         0;       % velocity (m/s)^
         0;
         0;
         0];      % angular velocity ^

x0_22 = [0;
         0;
         -1800;    % position (m)^
         0;
         0.02780;
         0;        % orientation (radians)^
         20.99;
         0;
         0.5837;   % velocity (m/s)^
         0;
         0;
         0];       % angular velocity ^

x0_23 = [0;
         0;
         -1800;    % position (m)^
         15;
         deg2rad(-12);
         deg2rad(270);      % orientation (degrees)^
         deg2rad(19);
         3;
         -2;       % velocity (m/s)^
         deg2rad(0.08);
         deg2rad(-0.2);
         0];       % angular velocity (deg/s)^

>>>>>>> Stashed changes
if Q2_1 == 1
    x0_21 = [0;
        0;
        -1609.34; % position ^
        0;
        0;
        0;       % orientation ^
        21;
        0;
        0;       % velocity ^
        0;
        0;
        0];      % angular velocity ^

    % wind in the inertial frame
    wind_inertial_21 = [0; 0; 0]; % Define wind vector in the inertial frame

    % control inputs vector
    aircraft_surfaces_21 = [0; 0; 0; 0]; % Define control inputs vector

    [t_part21, x_part21] = ode45(@(t,x) AircraftEOM(t,x,aircraft_surfaces_21,wind_inertial_21,aircraft_parameters), [0 200], x0_21);

    % make x_part21 a column vector
    x_part21 = x_part21';

    % apply constant aircraft surfaces over full time interval
    aircraft_surfaces_21 = repmat(aircraft_surfaces_21, 1, length(t_part21));

    % Generate Plots
    PlotAircraftSim(t_part21, x_part21, aircraft_surfaces_21, [1, 2, 3, 4, 5, 6], 'r');

end

if Q2_2 == 1
    x0_22 = [0;
        0;
        -1800; % position ^
        0;
        0.02780;
        0;       % orientation ^
        20.99;
        0;
        0.5837;       % velocity ^
        0;
        0;
        0];      % angular velocity ^

    % wind in the inertial frame
<<<<<<< Updated upstream
    wind_inertial_22 = [0; 0; 0]; % Define wind vector in the inertial frame
    
    % control inputs vector
    aircraft_surfaces_22 = [0.1079; 0; 0; 0.3182]; % Define control inputs vector
=======
    % wind_inertial = [0; 0; 0]; % Define wind vector in the inertial frame

    % control inputs vector
    % aircraft_surfaces = [0.1079; 0; 0; 0.3182]; % Define control inputs vector
>>>>>>> Stashed changes

    [t_part22, x_part22] = ode45(@(t,x) AircraftEOM(t,x,aircraft_surfaces_22,wind_inertial_22,aircraft_parameters), [0 200], x0_22);

    % make x_part21 a column vector
    x_part22 = x_part22';

    % apply constant aircraft surfaces over full time interval
    aircraft_surfaces_22 = repmat(aircraft_surfaces_22, 1, length(t_part22));

    % Generate Plots
    PlotAircraftSim(t_part22, x_part22, aircraft_surfaces_22, [7, 8, 9, 10, 11, 12], 'r');
end

if Q2_3 == 1
<<<<<<< Updated upstream
=======
    % wind in the inertial frame
    % wind_inertial = [0; 0; 0]; % Define wind vector in the inertial frame

    % control inputs vector
    % aircraft_surfaces = [0; 0; 0; 0]; % Define control inputs vector
>>>>>>> Stashed changes
    
    x0_23 = [0;
        0;
        -1800; % position ^
        deg2rad(15);
        deg2rad(-12);
        deg2rad(270);       % orientation ^
        19;
        3;
        -2;       % velocity ^
        deg2rad(0.08);
        deg2rad(-0.2);
        0];      % angular velocity ^


    % wind in the inertial frame
    wind_inertial_23 = [0; 0; 0]; % Define wind vector in the inertial frame
    
    % control inputs vector
    aircraft_surfaces_23 = [deg2rad(5); deg2rad(2); deg2rad(-13); 0.3]; % Define control inputs vector

    [t_part23, x_part23] = ode45(@(t,x) AircraftEOM(t,x,aircraft_surfaces_23,wind_inertial_23,aircraft_parameters), [0 200], x0_22);

    % make x_part21 a column vector
    x_part23 = x_part23';

    % apply constant aircraft surfaces over full time interval
    aircraft_surfaces_23 = repmat(aircraft_surfaces_23, 1, length(t_part23));

    % Generate Plots
    PlotAircraftSim(t_part23, x_part23, aircraft_surfaces_23, [13, 14, 15, 16, 17, 18], 'r');
end

%% Q3
if Q3_1 == 1
<<<<<<< Updated upstream
=======
    % Initial Values
    doublet_size = deg2rad(15); % (deg)
    doublet_time = 0.25; % (sec)

    % wind in the inertial frame
    wind_inertial = [0; 0; 0]; % Define wind vector in the inertial frame

    % control inputs vector
    aircraft_surfaces = [0.1079; 0; 0; 0.3182]; % Define control inputs vector

    % Short Period
    [t_part31, x_part31] = ode45(@(t,x) AircraftEOMDoublet(t, x, aircraft_surfaces, doublet_size,...
                                   doublet_time, wind_inertial, aircraft_parameters), [0.5 1.5], x0_22);
    x_part31 = x_part31'; % make x_part21 a column vector
    PlotAircraftSim(t_part31, x_part31, aircraft_surfaces, [1, 2, 3, 4, 5, 6], 'r'); % Generate Plots
>>>>>>> Stashed changes

end

if Q3_2 == 1
<<<<<<< Updated upstream
=======
    % Initial Values
    doublet_size = deg2rad(15); % (deg)
    doublet_time = 0.25; % (sec)

    % wind in the inertial frame
    wind_inertial = [0; 0; 0]; % Define wind vector in the inertial frame

    % control inputs vector
    aircraft_surfaces = [0.1079; 0; 0; 0.3182]; % Define control inputs vector

    % Phugoid
    [t_part31, x_part31] = ode45(@(t,x) AircraftEOMDoublet(t, x, aircraft_surfaces, doublet_size,...
                                   doublet_time, wind_inertial, aircraft_parameters), [1.5 100], x0_22);
    x_part31 = x_part31'; % make x_part21 a column vector
    PlotAircraftSim(t_part31, x_part31, aircraft_surfaces, [7, 8, 9, 10, 11, 12], 'r'); % Generate Plots
>>>>>>> Stashed changes

end
