function [fig, t] = mainProfiles(prof)
% mainProfiles(prof) 
%  draw post-processing plots.
%
% Input arguments
% ---------------
% prof : struct
%   data structure for the time profiles.

%% Load info
% Check that the relevant profiles were provided
if any( ~isfield(prof, {'vehSpd', 'battSOC', 'engSpd', 'engTrq', 'motSpd', 'motTrq'}) )
    error("The profiles structure must contain the fields: 'vehSpd', 'battSOC', 'engSpd', 'engTrq', 'motSpd', 'motTrq'.")
end

% Ensure all structures are scalar structures
prof = structArray2struct(prof);

time = 0:1:(length(prof.vehSpd)-1);
time = time(:);

%% Speed, with operating modes
fig = figure;
t = tiledlayout(4,1);
ax1 = nexttile;
grid on
hold on

OMs = ["pe", "cd", "bc"];
colors = ["#2ca02c", "#1f77b4", "#d62728"];
time_stairs = reshape( [ time(1:end-1)'; time(2:end)'-eps ], (length(time)-1)*2, 1 );
vehSpd_stairs = reshape( [ prof.vehSpd(1:end-1)'; prof.vehSpd(2:end)' ], (length(time)-1)*2, 1 );
OM_stairs = reshape( [ prof.opMode(1:end-1)'; prof.opMode(1:end-1)' ], (length(time)-1)*2, 1 );

% powerflowPFs(end-1:end) = [];
for n = 1:length(OMs)
    vehSpd.(OMs(n)) = vehSpd_stairs;
    vehSpd.(OMs(n))(~strcmp(OM_stairs, OMs(n))) = nan;
    plot(time_stairs, vehSpd.(OMs(n)), 'Color', colors(n), 'LineWidth', 1.5);
end

ylabel("Vehicle speed, m/s")
legend(OMs, 'Location', 'best')

%% SOC, engine and motor, fuel consumption
ax2 = nexttile;
plot(time, prof.battSOC, 'LineWidth', 1.5, 'Color', '#17becf')
grid on
ylabel("\sigma, -")

ax3 = nexttile;
hold on
plot(time, prof.engSpd .* prof.engTrq, 'LineWidth', 1.5, 'Color', '#d62728')
plot(time, prof.motSpd .* prof.motTrq, 'LineWidth', 1.5, 'Color', '#2ca02c')
legend("Engine", "Motor")
grid on
ylabel("Mech. power, W")

ax4 = nexttile;
plot(time, cumtrapz(time, prof.fuelFlwRate), 'LineWidth', 1.5, 'Color', '#8c564b')
grid on
ylabel("Fuel consumption, g")

xlabel(t, "Time, s", 'FontSize', 12)
linkaxes([ax1 ax2 ax3 ax4], 'x')
