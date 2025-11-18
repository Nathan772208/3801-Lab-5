function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% PLOTAIRCRAFTSIM plots the results of a full simulation once it is complete
% Inputs: 
%     time : time corresponding to nth set of state variables
%     aircraft_state_array : array of aircraft state vectors
%     control_input_array : array of control inputs
%     fig : vector of figure numbers to plot over
%     col : indicates plotting option for every plot
% 
% Outputs:
%     6 plots

% subplots for inertial position
figure(fig(1))
subplot(311)
plot(time, aircraft_state_array(1,:), col) 
hold on
grid on
title("x position")
xlabel("Time (s)")
ylabel("Inertial Position (m)")

subplot(312)
plot(time, aircraft_state_array(2,:), col)
hold on
grid on
title("y position")
xlabel("Time (s)")
ylabel("Inertial Position (m)")

subplot(313)
plot(time, aircraft_state_array(3,:), col)
hold on
grid on
title("z position")
xlabel("Time (s)")
ylabel("Inertial Position (m)")

sgtitle("Inertial Position of Aircraft Over Time")

% subplots for euler angles 
figure(fig(2))
subplot(311)
plot(time, aircraft_state_array(4,:), col)
hold on
grid on
title("Roll Angle \Phi")
xlabel("Time (s)")
ylabel("Euler Angles (rad)")

subplot(312)
plot(time, aircraft_state_array(5,:), col)
hold on
grid on
title("Pitch Angle \theta")
xlabel("Time (s)")
ylabel("Euler Angles (rad)")

subplot(313)
plot(time, aircraft_state_array(6,:), col)
hold on
grid on
title("Yaw Angle \Psi")
xlabel("Time (s)")
ylabel("Euler Angles (rad)")

sgtitle("Euler Angles of Aircraft Over Time")

% subplots for inertial velocity in body frame
figure(fig(3))
subplot(311)
plot(time, aircraft_state_array(7,:), col)
hold on
grid on
title("x velocity u^E")
xlabel("Time (s)")
ylabel("Inertial Velocity (m/s)")

subplot(312)
plot(time, aircraft_state_array(8,:), col)
hold on
grid on
title("y velocity v^E")
xlabel("Time (s)")
ylabel("Inertial Velocity (m/s)")

subplot(313)
plot(time, aircraft_state_array(9,:), col)
hold on
grid on
title("z velocity w^E")
xlabel("Time (s)")
ylabel("Inertial Velocity (m/s)")

sgtitle("Inertial Velocity of Aircraft in Body Frame Over Time")

% subplots for angular velocity
figure(fig(4))
subplot(311)
plot(time, aircraft_state_array(10,:), col)
hold on
grid on
title("\omega_x (p)")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")

subplot(312)
plot(time, aircraft_state_array(11,:), col)
hold on
grid on
title("\omega_y (q)")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")

subplot(313)
plot(time, aircraft_state_array(12,:), col)
hold on
grid on
title("\omega_z (r)")
xlabel("Time (s)")
ylabel("Angular Velocity (rad/s)")

sgtitle("Angular Velocity of Aircraft Over Time")

% subplots for each control variable
figure(fig(5))
subplot(411)
plot(time, rad2deg(control_input_array(1,:)), col)
hold on
grid on
title("Elevator Deflection")
xlabel("Time (s)")
ylabel("Deflection (deg)")

subplot(412)
plot(time, rad2deg(control_input_array(2,:)), col)
hold on
grid on
title("Aileron Deflection")
xlabel("Time (s)")
ylabel("Deflection (deg)")

subplot(413)
plot(time, rad2deg(control_input_array(3,:)), col)
hold on
grid on
title("Rudder Deflection")
xlabel("Time (s)")
ylabel("Deflection (deg)")

subplot(414)
plot(time, control_input_array(4,:), col)
hold on
grid on
title("Throttle")
xlabel("Time (s)")
ylabel("Fraction from 0 to 1")

sgtitle("Control Variables of Aircraft Over Time")

% three dimensional path of aircraft
figure(fig(6))
plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:), col, 'LineWidth',1);
hold on

% marking start point, end point of trajectory
plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), aircraft_state_array(3,1), 'g.', 'MarkerSize',15);
plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), aircraft_state_array(3,end), 'r.','MarkerSize',15);

% plot options
xlabel('x position (m)')
ylabel('y position (m)')
zlabel('z position (m)')
grid on
title('3D Path of Aircraft')
end