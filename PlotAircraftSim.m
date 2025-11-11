function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% ----------------------------------------------------------------------- %
% Inputs: time (s) ; aircraft state array ; control input array (N) ;
% figure numbers ; plotting option
%
% Outputs: 6 figures
%
% Methodology: Four figures with three subplots each showing state over
% time,mone figure with four subplots showing control inputs and one figure
% showing three dimensional path of the aircraft
%
% ----------------------------------------------------------------------- %


%% Figure 1 Inertial Position
figure(fig(1))

subplot(311)
hold on
plot(time,aircraft_state_array(:,1),col)
title("Inertial North position vs time")
xlabel("time (s)")
ylabel("N position (m)")

subplot(312)
hold on
plot(time,aircraft_state_array(:,2),col)
title("Inertial East position vs time")
xlabel("time (s)")
ylabel("E position (m)")

subplot(313)
hold on
plot(time,aircraft_state_array(:,3),col)
title("Inertial Down position vs time")
xlabel("time (s)")
ylabel("D position (m)")

%% Figure 2 Euler Angles
figure(fig(2))

subplot(311)
hold on
plot(time,aircraft_state_array(:,4),col)
title("Roll Angle vs time")
xlabel("time (s)")
ylabel("\phi (rad)")

subplot(312)
hold on
plot(time,aircraft_state_array(:,5),col)
title("Pitch Angle vs time")
xlabel("time (s)")
ylabel("\theta (rad)")

subplot(313)
hold on
plot(time,aircraft_state_array(:,6),col)
title("Yaw Angle vs time")
xlabel("time (s)")
ylabel("\psi (rad)")

%% Figure 3 Inertial Velocity
figure(fig(3))

subplot(311)
hold on
plot(time,aircraft_state_array(:,7),col)
title("Inertial velocity in body x direction vs time")
xlabel("time (s)")
ylabel("velocity (m/s)")

subplot(312)
hold on
plot(time,aircraft_state_array(:,8),col)
title("Inertial velocity in body y direction vs time")
xlabel("time (s)")
ylabel("velocity (m/s)")

subplot(313)
hold on
plot(time,aircraft_state_array(:,9),col)
title("Inertial velocity in body z direction vs time")
xlabel("time (s)")
ylabel("velocity (m/s)")

%% Figure 4 Angular Velocity
figure(fig(4))

subplot(311)
hold on
plot(time,aircraft_state_array(:,10),col)
title("Roll Direction Anglular Velocity vs time")
xlabel("time (s)")
ylabel("p (rad/s)")

subplot(312)
hold on
plot(time,aircraft_state_array(:,11),col)
title("Pitch Direction Anglular Velocity vs time")
xlabel("time (s)")
ylabel("q (rad/s)")

subplot(313)
hold on
plot(time,aircraft_state_array(:,12),col)
title("Yaw Direction Anglular Velocity vs time")
xlabel("time (s)")
ylabel("r (rad/s)")

%% Figure 5 Control Inputs
figure(fig(5))

subplot(221)
hold on
plot(time,rad2degree(control_input_array(:,1)),col)
title("Elevator Deflection vs time")
xlabel("time (s)")
ylabel("Elevator Deflection (deg)")

subplot(222)
hold on
plot(time,rad2degree(control_input_array(:,2)),col)
title("Aileron Deflection vs time")
xlabel("time (s)")
ylabel("Aileron Deflection (deg)")

subplot(223)
hold on
plot(time,rad2degree(control_input_array(:,3)),col)
title("Rudder Deflection vs time")
xlabel("time (s)")
ylabel("Rudder Deflection (deg)")

subplot(224)
hold on
plot(time,control_input_array(:,4),col)
title("Throttle Input vs time")
xlabel("time (s)")
ylabel("Throttle Amount (0=none 1=full)")
hold off

%% Figure 6 3D graph
figure(fig(6))

stateLength = size(aircraft_state_array,1);

hold on

% Create a colormap that transitions from green to red
cmap = [linspace(0, 1, stateLength)', linspace(1, 0, stateLength)', zeros(stateLength, 1)];

% Plot the 3D line with color gradient

for i = 1:stateLength-1
    plot3(aircraft_state_array(i:i+1,1), aircraft_state_array(i:i+1,2), aircraft_state_array(i:i+1,3),'Color',cmap(i,:),'LineWidth',2);
end

view(3)
grid on;
xlabel('N Inertial Direction');
ylabel('E Inertial Direction');
zlabel('D Inertial Direction');
title("3D Path Plot")

hold off


end