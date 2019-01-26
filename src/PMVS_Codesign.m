%--------------------------------------------------------------------------
% PMVS_Codesign.m
% Given an architecture, solve the corresponding co-design problem
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function [F,a,varargout] = PMVS_Codesign(p,Graph,varargin)

% try to obtain the co-design solution
try
    % number of function outputs
    nout = max(nargout,1);
    
    % optional starting point
    if ~isempty(varargin)
        X = varargin{1};
    end

    % create simscape model
    a = PMVS_CreateSimscapeModel(p,Graph);
    
    % create plant variable information
    [a,LB,UB] = PMVS_CreatePlantInfo(a);

    % solve the level p problem (plant outer loop in nested co-design)
    if nout > 2 % evaluate a single plant design and get the trajectories
        [a,F,O] = PMVS_OuterLoop(a,p,LB,UB,X);
        varargout{1} = O;
    else
        [a,F] = PMVS_OuterLoop(a,p,LB,UB);
    end
    
    % remove some fields that we no longer need
    a = rmfield(a,'mainpath');
    a = rmfield(a,'modelpath');
    a = rmfield(a,'sys');
    a = rmfield(a,'linearizeOpts');

catch ME
    
    % display the message
    disp(['error! ',ME.message])
    
    % save outputs
    a = []; % clear structure
    a.comp = ME.message; % save the error message
    a.xp = nan;
    a.xpopt = [];
    a.exitflag = 'error';

    % error occurred
    F = nan;
    
end

end