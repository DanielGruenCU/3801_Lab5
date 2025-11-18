function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
%% PlotAircraftSim - Plots the aircraft simulation data over time.
%
% Syntax: PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
%
% Inputs:
%   time                - A vector of time values (1D array).
% aircraft_state = [xi, yi, zi, roll, pitch, yaw, u, v, w, p, q, r]
%   NOTE: The function assumes the veolcity is the air relative velocity
%   vector.
%   control_input_array  - A matrix containing the control input data (4xN).
%   fig                 - A vector of figure handles for plotting.
%   col                 - A string or character vector specifying the color for the plots.
%
% Outputs:
%   None. The function generates plots for the aircraft simulation data.

% INPUTS:

    % Inertial Position
    f = figure(fig(1));
    f.Position = [0 400 600 500];
    tiledlayout(3,1)
    nexttile
    plot(time, aircraft_state_array(1, :), col) % x pos
    ylabel("x position (m)")
    title("Inertial Position vs. Time")
    
    nexttile
    plot(time, aircraft_state_array(2, :), col) % y pos
    ylabel("y position (m)")

    ax3 = nexttile;
    plot(time, aircraft_state_array(3, :), col) % z pos
    ylabel("z position (m)")
  
    ax3.YDir = 'reverse'; 


    xlabel("Time (s)")


    % Euler Angles
    f = figure(fig(2));
    f.Position = [0 0 600 500];
    tiledlayout(3,1)
    nexttile
    plot(time, aircraft_state_array(4, :), col)
    ylabel("Roll \Phi (rad)")
    title("Euler Angles vs. Time")
    
    nexttile
    plot(time, aircraft_state_array(5, :), col)
    ylabel("Pitch \Theta (rad)")

    nexttile
    plot(time, aircraft_state_array(6, :), col)
    ylabel("Yaw \Psi (rad)")

    xlabel("Time (s)")

    % Inertial Velocity
    f = figure(fig(3));
    f.Position = [600 400 600 500];
    tiledlayout(3,1)
    nexttile
    plot(time, aircraft_state_array(7, :), col)
    ylabel("u (m/s)")
    title("Inertial Velocity vs. Time")
    
    nexttile
    plot(time, aircraft_state_array(8, :), col)
    ylabel("v (m/s)")

    ax4 = nexttile;
    plot(time, aircraft_state_array(9, :), col)
    ylabel("w (m/s)")
    ax4.YDir = 'reverse';

    xlabel("Time (s)")

    % Angular Velocity
    f = figure(fig(4));
    f.Position = [600 0 600 500];
    tiledlayout(3,1)
    nexttile
    plot(time, aircraft_state_array(10, :), col)
    ylabel("p (rad/s)")
    title("Angular Velocity vs. Time")
    
    nexttile
    plot(time, aircraft_state_array(11, :), col)
    ylabel("q (rad/s)")

    nexttile
    plot(time, aircraft_state_array(12, :), col)
    ylabel("r (rad/s)")

    xlabel("Time (s)")
    
    % Control Variables
    f = figure(fig(5));
    f.Position = [1100 400 600 500];
    tiledlayout(4,1)
    nexttile
    plot(time, control_input_array(1, :), col) % Z_c
    ylabel("Elevator Deflection (deg)")
    title("Control Inputs vs. Time")

    nexttile
    plot(time, control_input_array(2, :), col) % L_c
    ylabel("Aileron Deflection (deg)")

    nexttile
    plot(time, control_input_array(3, :), col) % M_c
    ylabel("Rudder Elevator Deflection (deg)")

    nexttile
    plot(time, control_input_array(4, :), col) % N_c
    ylim([0 1])
    ylabel("Throttle (unitless)")
    xlabel("Time (s)")

    
    %% 3D Position
    f = figure(fig(6));
    f.Position = [1000 0 600 500];
    plot3(aircraft_state_array(1, :), aircraft_state_array(2, :), aircraft_state_array(3, :), col)
    title("3D Path of the Aircraft")
    xlabel("Inertial x-axis (North)")
    ylabel("Inertial y-axis (East)")
    zlabel("Inertial z-axis (Down)")
    set(gca, 'Zdir', 'reverse')
    zlim([-1900 -1000])

    hold on
    scatter3(aircraft_state_array(1, 1), aircraft_state_array(2, 1), aircraft_state_array(3, 1), 'MarkerEdgeColor','k', 'MarkerFaceColor', "green")
    scatter3(aircraft_state_array(1, end), aircraft_state_array(2, end), aircraft_state_array(3, end), 'MarkerEdgeColor','k', 'MarkerFaceColor', "red")
    

end