%--------------------------------------------------------------------------
% PMVS_CreatePlantInfo.m
% Given an architecture, determine some plant-related items such as the
% upper and lower bounds on the plant variables
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function [a,LB,UB] = PMVS_CreatePlantInfo(a)

% initialize plant variable index
index = 0;

% go through each component type
for idx = 1:length(a.comp)
    % determine if the component type has a plant variable
    switch a.comp(idx).name
        case 'K' % springs
            blockName = 'k';
            variableName = 'spr_rate';
            varMin = a.comp(3).min;
            varMax = a.comp(3).max;
        case 'B' % dampers
            blockName = 'b';
            variableName = 'D';
            varMin = a.comp(2).min;
            varMax = a.comp(2).max;
        case 'M' % masses
            blockName = 'm';
            variableName = 'mass';
            varMin = a.comp(1).min;
            varMax = a.comp(1).max;
        otherwise % empty since the component type doesn't have any xp
            blockName = [];
    end
    
    % create a listing of the variable variables
    if ~isempty(blockName)
        for k = find(a.comp(idx).I)
            index = index + 1; % plant variable index
            a.xp(index,1).blockName = [a.sys,'/',blockName,num2str(k)]; % block name
            a.xp(index,1).variableName = variableName; % variable name in block
            a.xp(index,1).min = varMin; % lower bound
            a.xp(index,1).max = varMax; % upper bound
        end
    end
    
end

% check if there are any plant variables
if isfield(a,'xp')
    % combine into single bounds vector
    LB = [a.xp.min];
    UB = [a.xp.max];
else % control-only problem
    a.xp = nan;
    LB = [];
    UB = [];
end

% set linearization options for the plant
a.linearizeOpts = linearizeOptions('SampleTime',0,...
    'LinearizationAlgorithm','numericalpert');

end