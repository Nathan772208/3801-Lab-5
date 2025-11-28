%% ASEN 3801 Lab 5
% Main Script

%% Housekeeping
clear; clc; close all;
ttwistor;

%% Question Toggles
Q2_1 = 1;
Q2_2 = 1;
Q2_3 = 1;
Q3_1 = 1;
Q3_2 = 1;

%% Q2
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

    % make x_part a column vector
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
    wind_inertial_22 = [0; 0; 0]; % Define wind vector in the inertial frame
    
    % control inputs vector
    aircraft_surfaces_22 = [0.1079; 0; 0; 0.3182]; % Define control inputs vector

    [t_part22, x_part22] = ode45(@(t,x) AircraftEOM(t,x,aircraft_surfaces_22,wind_inertial_22,aircraft_parameters), [0 200], x0_22);

    % make x_part a column vector
    x_part22 = x_part22';

    % apply constant aircraft surfaces over full time interval
    aircraft_surfaces_22 = repmat(aircraft_surfaces_22, 1, length(t_part22));

    % Generate Plots
    PlotAircraftSim(t_part22, x_part22, aircraft_surfaces_22, [7, 8, 9, 10, 11, 12], 'r');
end

if Q2_3 == 1
    
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

    [t_part23, x_part23] = ode45(@(t,x) AircraftEOM(t,x,aircraft_surfaces_23,wind_inertial_23,aircraft_parameters), [0 200], x0_23);

    % make x_part a column vector
    x_part23 = x_part23';

    % apply constant aircraft surfaces over full time interval
    aircraft_surfaces_23 = repmat(aircraft_surfaces_23, 1, length(t_part23));

    % Generate Plots
    PlotAircraftSim(t_part23, x_part23, aircraft_surfaces_23, [13, 14, 15, 16, 17, 18], 'r');
end

%% Q3
if Q3_1 == 1
    x0_31 = [0;
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
    wind_inertial_31 = [0; 0; 0]; % Define wind vector in the inertial frame
    
    % control inputs vector
    aircraft_surfaces_31 = [0.1079; 0; 0; 0.3182]; % Define control inputs vector

    % Doublet info
    doublet_size_31 = deg2rad(15);
    doublet_time_31 = 0.25;

    [t_part31, x_part31] = ode45(@(t,x) AircraftEOMDoublet(t,x,aircraft_surfaces_31,doublet_size_31, doublet_time_31,wind_inertial_31,aircraft_parameters), [0 3], x0_31);

    % make x_part a column vector
    x_part31 = x_part31';

    for i = 1:length(t_part31)
        aircraft_surfaces_31_vector(2,i) = aircraft_surfaces_31(2);
        aircraft_surfaces_31_vector(3,i) = aircraft_surfaces_31(3);
        aircraft_surfaces_31_vector(4,i) = aircraft_surfaces_31(4);

        if (t_part31(i) <= doublet_time_31)
            aircraft_surfaces_31_vector(1,i) = aircraft_surfaces_31(1) + doublet_size_31;
        end
        if (t_part31(i) > doublet_time_31 && t_part31(i) <= (2*doublet_time_31))
            aircraft_surfaces_31_vector(1,i) = aircraft_surfaces_31(1) - doublet_size_31;
        end
        if (t_part31(i) > (2*doublet_time_31))
            aircraft_surfaces_31_vector(1,i) = aircraft_surfaces_31(1);
        end
    end

    % Generate Plots
    PlotAircraftSim(t_part31, x_part31, aircraft_surfaces_31_vector, [19, 20, 21, 22, 23, 24], 'r');

    q = x_part31(11,:);              % q state (assuming order from lab handout)
    t = t_part31;

    % Focus on the response after the doublet has finished use just before
    % 0.5 to capture the first peak
    idx = t >= 0.45 & t <= 1.5;
    t_sp = t(idx);
    q_sp = q(idx);

    % Remove any tiny bias around trim
    q_trim = mean(q(end-10:end));
    y = q_sp - q_trim;

    % Find peaks of the oscillation
    [pk, tpk] = findpeaks(y, t_sp, 'MinPeakProminence', 1e-3);

    % Find approximate period
    idx_min = find(q_sp == min(q_sp));
    T_sp = abs(2*(tpk(1) - t_sp(idx_min)));

    % 1) Damped natural frequency from time between peaks
    omega_d = 2*pi / T_sp;

    % Find zeta using realtion between peaks
    A1 = abs(y(idx_min));   % magnitude of trough
    A2 = pk(1);             % magnitude of peak

    % Log decrement based on half-period amplitude change
    delta_half = abs(log(A1 / A2));
    delta      = 2 * delta_half;        % per full period

    % Damping ratio
    zeta = delta / sqrt(4*pi^2 + delta^2);

    % 3) Undamped natural frequency
    omega_n = omega_d / sqrt(1 - zeta^2);


end

if Q3_2 == 1
    [t_part32, x_part32] = ode45(@(t,x) AircraftEOMDoublet(t,x,aircraft_surfaces_31,doublet_size_31, doublet_time_31,wind_inertial_31,aircraft_parameters), [0 100], x0_31);
    
    % make x_part a column vector
    x_part32 = x_part32';

    for i = 1:length(t_part32)
        aircraft_surfaces_32_vector(2,i) = aircraft_surfaces_31(2);
        aircraft_surfaces_32_vector(3,i) = aircraft_surfaces_31(3);
        aircraft_surfaces_32_vector(4,i) = aircraft_surfaces_31(4);

        if (t_part32(i) <= doublet_time_31)
            aircraft_surfaces_32_vector(1,i) = aircraft_surfaces_31(1) + doublet_size_31;
        end
        if (t_part32(i) > doublet_time_31 && t_part32(i) <= (2*doublet_time_31))
            aircraft_surfaces_32_vector(1,i) = aircraft_surfaces_31(1) - doublet_size_31;
        end
        if (t_part32(i) > (2*doublet_time_31))
            aircraft_surfaces_32_vector(1,i) = aircraft_surfaces_31(1);
        end
    end

    % Generate Plots
    PlotAircraftSim(t_part32, x_part32, aircraft_surfaces_32_vector, [25, 26, 27, 28, 29, 30], 'r');

    % Find natural Frequency and damping ratio
    ue = x_part32(7,:);
    idx = t_part32 >= 5 & t_part32 <= 100; % Window where phugoid is clean with no short period
    t_ph = t_part32(idx);
    ue_ph = ue(idx);

    % subtract steady value so we are looking at deviation
    y_ph = ue_ph - ue(end);

    % find positive peaks of the phugoid
    [pk_ph, locs_ph] = findpeaks(y_ph, t_ph);
    
    % grab the clear peaks
    pk_ph = pk_ph(1:5);
    locs_ph = locs_ph(1:5);

    % Period of the phugoid and omega d
    T_ph = mean(diff(locs_ph));
    omega_d_ph = (2*pi)/T_ph;

    % Log decrement using successive peaks (up to the 5th peak) to find zeta
    delta_k = log(pk_ph(1:4) ./ pk_ph(2:5));
    delta = mean(delta_k);

    zeta_ph = delta / sqrt(4*pi^2 + delta^2);   % damping ratio

    % Natural Frequency
    omega_n_ph = omega_d_ph / sqrt(1 - zeta_ph^2);

end
