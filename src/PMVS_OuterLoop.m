%--------------------------------------------------------------------------
% PMVS_OuterLoop.m
% Solve the outer-loop problem in the nested co-design solution strategy
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function [a,F,varargout] = PMVS_OuterLoop(a,p,lb,ub,varargin)

% check if there are any plant variables
if isempty(lb)
    % obtain control-only solution
    if ~isempty(varargin)
        [F,O] = PMVS_Objective([],a,p,lb,ub);
        varargout{1} = O;
    else
        F = PMVS_Objective([],a,p,lb,ub);
    end
    
    % no plant variables
    a.xpopt = [];
    
    % save exit condition
    a.exitflag = 'control-only';
else
    % number of optimization variables
    nxp = length(lb);

    % ensure column vectors
    lb = lb(:); ub = ub(:);
    
    % scale (log10 scaling)
    lb = log10(lb);
    ub = log10(ub);
    
    % scale (0-1 linear scaling)
    LB = PMVS_Scaling(lb,lb,ub,1);
    UB = PMVS_Scaling(ub,lb,ub,1);
    
    % initialize plant design
    if ~isempty(varargin) % provided as an input
        X = PMVS_Scaling(log10(varargin{1}),lb,ub,1);
    else
%         X = LB + (UB-LB).*rand(nxp,1); % random points
        X = LB + (UB-LB).*ones(nxp,1)/2; % middle point
    end

    % check if we only want to evaluate a given X
    if ~isempty(varargin)
        % evaluate the inner-loop problem
        [F,O] = PMVS_Objective(X,a,p,lb,ub);
        varargout{1} = O;
        
        EXITFLAG = nan;
    else
    %----------------------------------------------------------------------
    % START: outer-loop global search
    %----------------------------------------------------------------------
    % patternsearch options
    options = psoptimset('Display','Iter','MaxIter',10,'Cache','on',...
        'UseParallel',false,'CompletePoll','on','CompleteSearch','on',...
        'SearchMethod', {@searchlhs,1,nxp*30});

    % global search method
    X = patternsearch(@(x) PMVS_Objective(x,a,p,lb,ub),X,[],[],[],[],LB,UB,[],options);

    %----------------------------------------------------------------------
    % END: outer-loop global search
    %----------------------------------------------------------------------

    %----------------------------------------------------------------------
    % START: outer-loop local search
    %----------------------------------------------------------------------
    % fmincon optimization
    options = optimoptions('fmincon','Display','Iter','Algorithm','sqp',...
        'UseParallel',false,'MaxIter',20,...
        'FiniteDifferenceStepSize',1000*sqrt(eps),'OptimalityTolerance',1e-4);

    % local search method
    [X,F,EXITFLAG] = fmincon(@(x) PMVS_Objective(x,a,p,lb,ub),X,[],[],[],[],LB,UB,[],options);

    %----------------------------------------------------------------------
    % END: outer-loop local search
    %----------------------------------------------------------------------
    end
    
    % unscale from 0-1 linear mapping
    X = PMVS_Scaling(X,lb,ub,2); 

    % unscale from log10 mapping
    X = 10.^(X);

    % add unscaled plant variables
    a.xpopt = X;
    
    % save exit condition
    a.exitflag = EXITFLAG;
end

% display objective function value
% disp(F)

end