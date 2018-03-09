function [x_next, y_next] = calcvel(x_curr,y_curr, vx_curr, vy_curr, t_curr, t_next)
%CALCVEL Calculates the velocity and the displacement for the next time
%instant
%   This function integrates the velocities to give out positions and the
%   next time instant velocity
delx = vx_curr*(t_next - t_curr);
dely = vy_curr*(t_next - t_curr);
x_next = x_curr + delx;
y_next = y_curr + dely;
end

