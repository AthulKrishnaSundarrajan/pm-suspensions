%--------------------------------------------------------------------------
% PMVS_UpdateMatrices.m
% Given an architecture and plant variable values, update the linear
% state-space model matrices
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function [A,B,C,D] = PMVS_UpdateMatrices(a,xp)

% update plant variables in model
for idx = 1:length(xp)
    % get the block name
    blockName = a.xp(idx).blockName;
    
    % get the variable name in the block
    variableName = a.xp(idx).variableName;
    
    % convert to string
    value = num2str(xp(idx));
    
    % set the parameter
    set_param(blockName,variableName,value);
end

% save the system
% save_system(a.sys)

% linearize model
try
    linsys = linearize(a.sys,a.linearizeOpts);
catch
    error('Error: linearization failed')
end

% extract matrices
A = linsys.A;
B = linsys.B;
C = linsys.C;
D = linsys.D;

end