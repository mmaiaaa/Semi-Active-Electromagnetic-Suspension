function [T, z_g, z_gdot] = road_profile_half_sine()
% ROAD_PROFILE_HALF_SINE

dt = 0.001;
T  = (0:dt:5)';

% Speed bump parameters
h = 0.089;        % bump height (m)
L = 0.375;        % bump length (m)
v = 2;            % vehicle speed (m/s)

% Bump duration
t_bump = 2*L/v;

% Road displacement (half-sine squared)
z_g = h * (sin(pi*v*T/(2*L))).^2 .* (T <= t_bump);

% Road velocity (finite difference)
z_gdot = [0; diff(z_g)/dt];
end
