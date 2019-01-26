%--------------------------------------------------------------------------
% PMVS_Objective.m
% Compute the objective for a given architecture and plant design. This
% needs to solve the inner-loop control problem.
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function [F,varargout] = PMVS_Objective(xp,a,p,lb,ub)

% number of function outputs
nout = max(nargout,1);

% unscale from linear 0 to 1 mapping
xp = PMVS_Scaling(xp,lb,ub,2); 

% unscale from log mapping
xp = 10.^(xp);

% update matrices
[A,B,C,D] = PMVS_UpdateMatrices(a,xp);
    
% find inner-loop solution
if nout > 1 % evaluate a single plant design and get the trajectories
    [F,O] = PMVS_InnerLoop(p,A,B,C,D);
    varargout{1} = O;
else
    F = PMVS_InnerLoop(p,A,B,C,D);
end

end