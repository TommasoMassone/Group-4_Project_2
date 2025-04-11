function [fig, ax] = emMapWithPF(em, prof, emType, opModes)
arguments
    em struct
    prof struct
    emType string {mustBeText, mustBeMember(emType, ["mot", "gen"])}  = "mot"
    opModes string {mustBeText, mustBeMember(opModes, ["all", "pe", "cd", "bc"])}  = ["all"]
end
%emMapWithPF
% Plot the e-machine efficiency map and the operating points, colored based
% on the powerflow.
%
% Input arguments
% ---------------
% em : struct
%   E-machine data structure.
% prof : struct
%   Time profiles data structure.
% powerflows : string, optional
%   Specify "mot" for the motor operating points or "gen" for the
%   generator. The default is "mot".
% opModes : string, optional
%   Specify one or more operating modes in a string array. Specify:
%       "pe"  for pure electric;
%       "cd"  for charge depleting;
%       "bc"  for battery charging;
%       "all" for all OMs.
%   The default is ["cd", "bc"] for the generator and ["pe", "cd", "bc"] for the motor.
%
% Outputs
% ---------------
% fig : Figure
%   Figure handle of the plot.
% ax : Axes
%   Axes handle of the plot.

%% Draw the EM efficiency map
[fig, ax] = emMapPlot(em);

% Ensure the prof structures is a scalar structure
prof = structArray2struct(prof);

% Check and retrieve the relevant profiles
switch emType
    case 'mot'
        if any( ~isfield(prof, {'motSpd', 'motTrq'}) )
            error("The profiles structure must contain the fields: 'motSpd', 'motTrq'.")
        end
        spd = prof.motSpd;
        trq = prof.motTrq;

        if strcmp(opModes, "all")
            opModes = ["pe", "cd", "bc"];
        end
    case 'gen'
        if any( ~isfield(prof, {'genSpd', 'genTrq'}) )
            error("The profiles structure must contain the fields: 'genSpd', 'genTrq'.")
        end
        spd = prof.genSpd;
        trq = prof.genTrq;
        
        if strcmp(opModes, "all")
            opModes = ["cd", "bc"];
        end
end

%% Operating points scatter plot
for n = 1:length(opModes)
    OM = opModes(n);
    idx = ismember(prof.opMode, OM);
    switch OM
        case 'pe'
            color = 'g';
        case 'cd'
            color = 'b';
        case 'bc'
            color = 'r';
        otherwise
            continue
    end

    s = scatter(spd(idx) .* 30/pi, trq(idx), color, 'x', 'DisplayName', OM);

    % Custom datatip
    s.DataTipTemplate.DataTipRows(1).Label = 'Speed';
    s.DataTipTemplate.DataTipRows(2).Label = 'Torque';
    time = 0:1:(length(spd)-1);
    s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('Time, s', time);
end


%% Finalize figure
if strcmp(emType, 'gen')
    ax.YDir = 'reverse';
end

title(emType + " operating points")

end