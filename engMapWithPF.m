function engMapWithPF(eng, prof, contour_type)
arguments
    eng struct
    prof struct
    contour_type {mustBeTextScalar, mustBeMember(contour_type, ["fc", "bsfc", "eff"])}  = "bsfc"
end
%engMapPlot
% Plot the engine fuel consumption, brake specific fuel consumption or
% efficiency map.
%
% Input arguments
% ---------------
% eng : struct
%   Engine data structure.
% prof : struct
%   Time profiles data structure.
% contour_type : string, optional
%   Specify: "fc" to plot the fuel flow rate map;
%            "bsfc" (default) for brake specific fuel consumption;
%            "eff" for fuel conversion efficiency.
%
% Outputs
% ---------------
% fig : Figure
%   Figure handle of the plot.
% ax : Axes
%   Axes handle of the plot.
%


%% Plot engine map
[fig, ax] = engMapPlot(eng, contour_type);

% Check that the relevant profiles were provided
if any( ~isfield(prof, {'engSpd', 'engTrq'}) )
    error("The profiles structure must contain the fields: 'engSpd', 'engTrq' .")
end

% Ensure prof is a scalar structure
prof = structArray2struct(prof);

%% Operating points scatter plot
s = scatter(prof.engSpd .* 30/pi, prof.engTrq, 'r', 'x', 'DisplayName', 'Operating points');

% Custom datatip
s.DataTipTemplate.DataTipRows(1).Label = 'Speed';
s.DataTipTemplate.DataTipRows(2).Label = 'Torque';
time = 0:1:(length(prof.engSpd)-1);
s.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow('Time, s', time);

%% Finalize figure
title('Engine operating points')

end