function fig = powerProfiles(prof, components)
arguments
    prof struct
    components string {mustBeText, mustBeMember(components, ["all", "veh",  "mot", "mot_mech", "mot_el", "eng", "gen", "gen_mech", "gen_el", "batt",])}  = ["mot_mech", "eng", "batt"]
end
%simulationAnalysis 
% draw post-processing plots
%
% Input arguments
% ---------------
% prof : struct
%   data structures for the time profiles.
% components : string, optional
%   Specify one or more components to represent in a string array. Specify:
%       "veh" for the vehicle level (driving force * vehicle speed);
%       "eng" for the engine;
%       "gen_mech" for the generator (mechanical power);
%       "gen_el" for the generator (electrical power);
%       "mot_mech" for the motor (mechanical power);
%       "mot_el" for the motor (electrical power);
%       "batt" for the battery;
%       "all" for all components;
%   The default is ["mot_mech", "eng", "batt"].

%% Set default values for components
if ismember("all", components)
    components = ["veh", "eng", "mot_el", "gen_el", "batt"];
end
if ismember("mot", components)
    components( ismember(components, "mot") ) = "mot_mech";
end
if ismember("gen", components)
    components( ismember(components, "gen") ) = "gen_el";
end

% Ensure all structures are scalar structures
prof = structArray2struct(prof);

time = 0:1:(length(prof.vehSpd)-1);
time = time(:);

%% Speed profile, with operating modes
OMs = ["pe", "cd", "bc"];
colors = ["#2ca02c", "#1f77b4", "#d62728"];
time_stairs = reshape( [ time(1:end-1)'; time(2:end)'-eps ], (length(time)-1)*2, 1 );
vehSpd_stairs = reshape( [ prof.vehSpd(1:end-1)'; prof.vehSpd(2:end)' ], (length(time)-1)*2, 1 );
OM_stairs = reshape( [ prof.opMode(1:end-1)'; prof.opMode(1:end-1)' ], (length(time)-1)*2, 1 );

fig = figure;
t = tiledlayout(2,1);

ax1 = nexttile;
grid on
hold on

% powerflowPFs(end-1:end) = [];
for n = 1:length(OMs)
    vehSpd.(OMs(n)) = vehSpd_stairs;
    vehSpd.(OMs(n))(~strcmp(OM_stairs, OMs(n))) = nan;
    plot(time_stairs, vehSpd.(OMs(n)), 'Color', colors(n), 'LineWidth', 1.5);
end

ylabel("Vehicle speed, m/s")
legend(OMs, 'Location', 'best')

%% Power profiles
ax2 = nexttile;
hold on
grid on

if ismember("veh", components)
    % Check that the relevant profiles were provided
    if any( ~isfield(prof, {'vehSpd', 'vehForce'}) )
        error("The profiles structure must contain the fields: 'vehSpd', 'vehForce'.")
    end
    % Plot
    plot(time, prof.vehSpd .* prof.vehForce, 'LineWidth', 1.5, 'Color', '#7f7f7f', 'DisplayName', "vehicle")
end

if ismember("mot_mech", components)
    % Check that the relevant profiles were provided
    if any( ~isfield(prof, {'motSpd', 'motTrq'}) )
        error("The profiles structure must contain the fields: 'motSpd', 'motTrq'.")
    end
    % Plot
    plot(time, prof.motSpd .* prof.motTrq, 'LineWidth', 1.5, 'Color', '#2ca02c', 'DisplayName', "mot (mechanical)")
end

if ismember("mot_el", components)
    % Check that the relevant profiles were provided
    if any( ~isfield(prof, 'motElPwr') )
        error("The profiles structure must contain the fields: 'motElPwr'.")
    end
    % Plot
    plot(time, prof.motElPwr, 'LineWidth', 1.5, 'Color', '#bcbd22', 'DisplayName', "mot (electrical)")
end

if ismember("eng", components)
    % Check that the relevant profiles were provided
    if any( ~isfield(prof, {'engSpd', 'engTrq'}) )
        error("The profiles structure must contain the fields: 'engSpd', 'engTrq'.")
    end
    % Plot
    plot(time, prof.engSpd .* prof.engTrq, 'LineWidth', 1.5, 'Color', '#d62728', 'DisplayName', "engine")
end

if ismember("gen_mech", components)
    % Check that the relevant profiles were provided
    if any( ~isfield(prof, {'genSpd', 'genTrq'}) )
        error("The profiles structure must contain the fields: 'genSpd', 'genTrq'.")
    end
    % Plot
    plot(time, prof.genSpd .* prof.genTrq, 'LineWidth', 1.5, 'Color', '#e377c2', 'DisplayName', "gen (mechanical)")
end

if ismember("gen_el", components)
    % Check that the relevant profiles were provided
    if any( ~isfield(prof, 'genElPwr') )
        error("The profiles structure must contain the fields: 'genElPwr'.")
    end
    % Plot
    plot(time, prof.genElPwr, 'LineWidth', 1.5, 'Color', '#9467bd', 'DisplayName', "gen (electrical)")
end

if ismember("batt", components)
    % Check that the relevant profiles were provided
    if any( ~isfield(prof, {'battVolt', 'battCurr'}) )
        error("The profiles structure must contain the fields: 'battVolt', 'battCurr'.")
    end
    % Plot
    plot(time, prof.battVolt .* prof.battCurr, 'LineWidth', 1.5, 'Color', '#1f77b4', 'DisplayName', "battery")
end

ylabel("Power, W")
legend('Location', 'best')

xlabel(t, "Time, s", 'FontSize', 9)
linkaxes([ax1,ax2], 'x')